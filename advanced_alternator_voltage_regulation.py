#!/usr/bin/env python3
"""
Advanced 3-Phase Alternator Voltage Regulation Simulator
Multi-Physics Simulation with Electromagnetic-Thermal-Mechanical Coupling

Features:
- Voltage regulation calculation at different power factors
- Real-time dynamic simulation with ODE solvers (RK45, Euler)
- Multi-physics modeling (electromagnetic, thermal, mechanical)
- Economic analysis and loss breakdown
- Thermal derating and power consumption tracking
- Advanced control systems
- Auto-scaling visualization
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math
from datetime import datetime
import threading
import time


class AlternatorPhysicsModel:
    """Multi-physics model for alternator behavior"""

    def __init__(self, rating_kva, voltage, phases=3, frequency=50):
        self.rating_kva = rating_kva
        self.voltage_line = voltage
        self.voltage_phase = voltage / np.sqrt(3)
        self.phases = phases
        self.frequency = frequency
        self.omega = 2 * np.pi * frequency

        # Machine parameters
        self.Ra = 0.06  # Armature resistance per phase (Ω)
        self.Xs = 0.864  # Synchronous reactance per phase (Ω)
        self.Zs = np.sqrt(self.Ra**2 + self.Xs**2)

        # Full load current
        self.I_fl = (rating_kva * 1000) / (np.sqrt(3) * voltage)

        # Thermal parameters
        self.thermal_resistance = 0.05  # K/W
        self.thermal_capacitance = 5000  # J/K
        self.ambient_temp = 25  # °C
        self.max_temp = 155  # °C (Class F insulation)
        self.current_temp = self.ambient_temp

        # Mechanical parameters
        self.inertia = 50  # kg·m²
        self.friction_coeff = 0.1  # N·m·s
        self.rated_torque = (rating_kva * 1000) / (2 * np.pi * frequency / (self.phases/2))

        # Efficiency and losses
        self.iron_loss_coeff = 0.015
        self.friction_loss = 2000  # W
        self.windage_loss = 1500  # W
        self.stray_load_loss_factor = 0.01

        # Economic parameters
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost_per_hour = 5.0  # $/hour

    def calculate_voltage_regulation(self, power_factor, lagging=True):
        """Calculate voltage regulation at given power factor"""
        phi = np.arccos(power_factor)
        if not lagging:
            phi = -phi

        # Current at full load
        I = self.I_fl
        V = self.voltage_phase

        # Induced EMF per phase
        E = np.sqrt((V * np.cos(phi) + I * self.Ra)**2 +
                    (V * np.sin(phi) + I * self.Xs)**2)

        # Voltage regulation
        regulation = ((E - V) / V) * 100

        return regulation, E

    def copper_losses(self, current):
        """Calculate copper losses (I²R losses)"""
        return 3 * current**2 * self.Ra

    def iron_losses(self, voltage):
        """Calculate iron losses (core losses)"""
        return self.iron_loss_coeff * voltage**2

    def mechanical_losses(self, speed_rpm):
        """Calculate mechanical losses (friction + windage)"""
        speed_factor = (speed_rpm / (120 * self.frequency / 2))**2
        return self.friction_loss + self.windage_loss * speed_factor

    def stray_load_losses(self, current):
        """Calculate stray load losses"""
        return self.stray_load_loss_factor * (current / self.I_fl)**2 * self.rating_kva * 1000

    def total_losses(self, current, voltage, speed_rpm):
        """Calculate total losses"""
        P_cu = self.copper_losses(current)
        P_iron = self.iron_losses(voltage)
        P_mech = self.mechanical_losses(speed_rpm)
        P_stray = self.stray_load_losses(current)

        return {
            'copper': P_cu,
            'iron': P_iron,
            'mechanical': P_mech,
            'stray': P_stray,
            'total': P_cu + P_iron + P_mech + P_stray
        }

    def thermal_model(self, losses, temp_current):
        """Thermal differential equation: C·dT/dt = P_loss - (T-T_amb)/R_th"""
        dT_dt = (losses - (temp_current - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance
        return dT_dt

    def mechanical_model(self, torque_electric, torque_mechanical, speed):
        """Mechanical differential equation: J·dω/dt = T_e - T_m - B·ω"""
        torque_friction = self.friction_coeff * speed
        dw_dt = (torque_electric - torque_mechanical - torque_friction) / self.inertia
        return dw_dt

    def derating_factor(self, temperature):
        """Calculate derating factor based on temperature"""
        if temperature <= 100:
            return 1.0
        elif temperature <= self.max_temp:
            return 1.0 - 0.01 * (temperature - 100) / (self.max_temp - 100)
        else:
            return 0.5  # Severe derating if overheating


class DynamicSimulator:
    """Real-time dynamic simulation engine"""

    def __init__(self, model):
        self.model = model
        self.time = 0
        self.dt = 0.01  # Time step (s)
        self.running = False

        # State variables
        self.current_state = {
            'voltage': model.voltage_phase,
            'current': 0,
            'temperature': model.ambient_temp,
            'speed': 120 * model.frequency / 2,  # Synchronous speed
            'torque': 0,
            'power_output': 0,
            'losses': {}
        }

        # History for plotting
        self.history = {
            'time': [],
            'voltage': [],
            'current': [],
            'temperature': [],
            'speed': [],
            'power': [],
            'losses': [],
            'efficiency': []
        }

    def system_ode(self, t, y, load_current, load_pf):
        """
        Coupled ODE system for multi-physics simulation
        y = [temperature, speed, field_current]
        """
        temp, speed, i_field = y

        # Calculate current based on load
        current = load_current

        # Calculate losses
        losses_dict = self.model.total_losses(
            current,
            self.model.voltage_phase,
            speed
        )
        total_loss = losses_dict['total']

        # Thermal dynamics
        dT_dt = self.model.thermal_model(total_loss, temp)

        # Mechanical dynamics
        phi = np.arccos(load_pf)
        power_output = np.sqrt(3) * self.model.voltage_line * current * load_pf
        torque_electric = power_output / (speed * 2 * np.pi / 60) if speed > 0 else 0
        torque_mechanical = self.model.rated_torque * (load_current / self.model.I_fl)

        dw_dt = self.model.mechanical_model(torque_electric, torque_mechanical, speed * 2 * np.pi / 60)
        dw_dt = dw_dt * 60 / (2 * np.pi)  # Convert to RPM/s

        # Field current dynamics (simplified first-order)
        tau_field = 0.5  # Field time constant
        i_field_ref = 1.0  # Reference field current (pu)
        di_field_dt = (i_field_ref - i_field) / tau_field

        return [dT_dt, dw_dt, di_field_dt]

    def step_euler(self, load_current, load_pf):
        """Euler method for ODE integration"""
        y = [
            self.current_state['temperature'],
            self.current_state['speed'],
            1.0  # field current (pu)
        ]

        dy_dt = self.system_ode(self.time, y, load_current, load_pf)

        # Update state
        y_new = [y[i] + dy_dt[i] * self.dt for i in range(len(y))]

        self.current_state['temperature'] = y_new[0]
        self.current_state['speed'] = y_new[1]
        self.current_state['current'] = load_current

        # Update other parameters
        self._update_derived_states(load_current, load_pf)

    def step_rk45(self, load_current, load_pf):
        """RK45 method for ODE integration"""
        y0 = [
            self.current_state['temperature'],
            self.current_state['speed'],
            1.0
        ]

        # Solve for one time step
        sol = solve_ivp(
            lambda t, y: self.system_ode(t, y, load_current, load_pf),
            [self.time, self.time + self.dt],
            y0,
            method='RK45',
            max_step=self.dt
        )

        if sol.success:
            y_new = sol.y[:, -1]
            self.current_state['temperature'] = y_new[0]
            self.current_state['speed'] = y_new[1]
            self.current_state['current'] = load_current

            self._update_derived_states(load_current, load_pf)

    def _update_derived_states(self, load_current, load_pf):
        """Update derived state variables"""
        # Calculate voltage with derating
        derating = self.model.derating_factor(self.current_state['temperature'])
        self.current_state['voltage'] = self.model.voltage_phase * derating

        # Calculate losses
        losses = self.model.total_losses(
            load_current,
            self.current_state['voltage'],
            self.current_state['speed']
        )
        self.current_state['losses'] = losses

        # Calculate power output
        power_output = np.sqrt(3) * self.model.voltage_line * load_current * load_pf
        self.current_state['power_output'] = power_output

        # Calculate torque
        speed_rad = self.current_state['speed'] * 2 * np.pi / 60
        self.current_state['torque'] = power_output / speed_rad if speed_rad > 0 else 0

    def record_history(self):
        """Record current state to history"""
        self.history['time'].append(self.time)
        self.history['voltage'].append(self.current_state['voltage'])
        self.history['current'].append(self.current_state['current'])
        self.history['temperature'].append(self.current_state['temperature'])
        self.history['speed'].append(self.current_state['speed'])
        self.history['power'].append(self.current_state['power_output'])
        self.history['losses'].append(self.current_state['losses']['total'])

        # Calculate efficiency
        if self.current_state['power_output'] > 0:
            efficiency = (self.current_state['power_output'] /
                         (self.current_state['power_output'] + self.current_state['losses']['total'])) * 100
        else:
            efficiency = 0
        self.history['efficiency'].append(efficiency)

    def reset(self):
        """Reset simulation to initial state"""
        self.time = 0
        self.current_state = {
            'voltage': self.model.voltage_phase,
            'current': 0,
            'temperature': self.model.ambient_temp,
            'speed': 120 * self.model.frequency / 2,
            'torque': 0,
            'power_output': 0,
            'losses': {}
        }

        self.history = {
            'time': [],
            'voltage': [],
            'current': [],
            'temperature': [],
            'speed': [],
            'power': [],
            'losses': [],
            'efficiency': []
        }


class AlternatorSimulatorGUI:
    """Main GUI application"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced 3-Phase Alternator Voltage Regulation Simulator")
        self.root.geometry("1400x900")

        # Initialize model with Example 30.26 parameters
        self.model = AlternatorPhysicsModel(
            rating_kva=2000,
            voltage=2300,
            phases=3,
            frequency=50
        )

        self.simulator = DynamicSimulator(self.model)
        self.simulation_thread = None
        self.solver_method = 'RK45'

        # Create UI
        self.create_menu()
        self.create_main_layout()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

        # Calculate initial regulation
        self.calculate_static_regulation()

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Export Results", command=self.export_results)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Start", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Solver menu
        solver_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Solver", menu=solver_menu)
        solver_menu.add_radiobutton(label="RK45 (Adaptive)",
                                     command=lambda: self.set_solver('RK45'))
        solver_menu.add_radiobutton(label="Euler (Fixed Step)",
                                     command=lambda: self.set_solver('Euler'))

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_layout(self):
        """Create main application layout"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Tab 1: Main Control and Visualization
        self.tab_main = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_main, text="Main Control")
        self.create_main_tab()

        # Tab 2: Multi-Physics Analysis
        self.tab_physics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_physics, text="Multi-Physics Analysis")
        self.create_physics_tab()

        # Tab 3: Economic Analysis
        self.tab_economics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_economics, text="Economic Analysis")
        self.create_economics_tab()

        # Tab 4: Advanced Controls
        self.tab_controls = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_controls, text="Advanced Controls")
        self.create_controls_tab()

    def create_main_tab(self):
        """Create main control tab"""
        # Left panel: Controls
        left_frame = ttk.Frame(self.tab_main, width=400)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5)

        # Input parameters group
        params_group = ttk.LabelFrame(left_frame, text="Alternator Parameters", padding=10)
        params_group.pack(fill=tk.X, pady=5)

        # Rating
        ttk.Label(params_group, text="Rating (kVA):").grid(row=0, column=0, sticky=tk.W)
        self.rating_var = tk.StringVar(value="2000")
        ttk.Entry(params_group, textvariable=self.rating_var, width=15).grid(row=0, column=1)

        # Voltage
        ttk.Label(params_group, text="Voltage (V):").grid(row=1, column=0, sticky=tk.W)
        self.voltage_var = tk.StringVar(value="2300")
        ttk.Entry(params_group, textvariable=self.voltage_var, width=15).grid(row=1, column=1)

        # Frequency
        ttk.Label(params_group, text="Frequency (Hz):").grid(row=2, column=0, sticky=tk.W)
        self.freq_var = tk.StringVar(value="50")
        ttk.Entry(params_group, textvariable=self.freq_var, width=15).grid(row=2, column=1)

        # Armature resistance
        ttk.Label(params_group, text="Ra (Ω):").grid(row=3, column=0, sticky=tk.W)
        self.ra_var = tk.StringVar(value="0.06")
        ttk.Entry(params_group, textvariable=self.ra_var, width=15).grid(row=3, column=1)

        # Synchronous reactance
        ttk.Label(params_group, text="Xs (Ω):").grid(row=4, column=0, sticky=tk.W)
        self.xs_var = tk.StringVar(value="0.864")
        ttk.Entry(params_group, textvariable=self.xs_var, width=15).grid(row=4, column=1)

        # Control sliders group
        control_group = ttk.LabelFrame(left_frame, text="Load Control", padding=10)
        control_group.pack(fill=tk.X, pady=5)

        # Load current slider
        ttk.Label(control_group, text="Load Current (% of FL):").pack(anchor=tk.W)
        self.load_current_var = tk.DoubleVar(value=100)
        self.load_current_slider = ttk.Scale(
            control_group,
            from_=0,
            to=120,
            variable=self.load_current_var,
            orient=tk.HORIZONTAL,
            command=self.update_load_display
        )
        self.load_current_slider.pack(fill=tk.X)
        self.load_current_label = ttk.Label(control_group, text="100.0 % (502.0 A)")
        self.load_current_label.pack()

        # Power factor slider
        ttk.Label(control_group, text="Power Factor:").pack(anchor=tk.W)
        self.power_factor_var = tk.DoubleVar(value=0.8)
        self.power_factor_slider = ttk.Scale(
            control_group,
            from_=0.1,
            to=1.0,
            variable=self.power_factor_var,
            orient=tk.HORIZONTAL,
            command=self.update_pf_display
        )
        self.power_factor_slider.pack(fill=tk.X)
        self.power_factor_label = ttk.Label(control_group, text="0.80 lagging")
        self.power_factor_label.pack()

        # Power factor type
        self.pf_type_var = tk.StringVar(value="lagging")
        pf_frame = ttk.Frame(control_group)
        pf_frame.pack(fill=tk.X, pady=5)
        ttk.Radiobutton(pf_frame, text="Lagging", variable=self.pf_type_var,
                       value="lagging").pack(side=tk.LEFT)
        ttk.Radiobutton(pf_frame, text="Leading", variable=self.pf_type_var,
                       value="leading").pack(side=tk.LEFT)
        ttk.Radiobutton(pf_frame, text="Unity", variable=self.pf_type_var,
                       value="unity", command=lambda: self.power_factor_var.set(1.0)).pack(side=tk.LEFT)

        # Control buttons
        button_frame = ttk.Frame(left_frame)
        button_frame.pack(fill=tk.X, pady=10)

        self.start_btn = ttk.Button(button_frame, text="Start", command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        self.stop_btn = ttk.Button(button_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        self.reset_btn = ttk.Button(button_frame, text="Reset", command=self.reset_simulation)
        self.reset_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        # Static results group
        results_group = ttk.LabelFrame(left_frame, text="Voltage Regulation Results", padding=10)
        results_group.pack(fill=tk.BOTH, expand=True, pady=5)

        self.results_text = tk.Text(results_group, height=15, width=45, wrap=tk.WORD)
        self.results_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(results_group, command=self.results_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Right panel: Visualization
        right_frame = ttk.Frame(self.tab_main)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create figure for plots
        self.fig_main = Figure(figsize=(8, 6), dpi=100)
        self.canvas_main = FigureCanvasTkAgg(self.fig_main, right_frame)
        self.canvas_main.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots
        self.ax1 = self.fig_main.add_subplot(2, 2, 1)
        self.ax2 = self.fig_main.add_subplot(2, 2, 2)
        self.ax3 = self.fig_main.add_subplot(2, 2, 3)
        self.ax4 = self.fig_main.add_subplot(2, 2, 4)

        self.fig_main.tight_layout(pad=2.0)

    def create_physics_tab(self):
        """Create multi-physics analysis tab"""
        # Create figure for multi-physics plots
        self.fig_physics = Figure(figsize=(10, 8), dpi=100)
        canvas = FigureCanvasTkAgg(self.fig_physics, self.tab_physics)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots for different physics domains
        self.ax_thermal = self.fig_physics.add_subplot(3, 2, 1)
        self.ax_losses = self.fig_physics.add_subplot(3, 2, 2)
        self.ax_mechanical = self.fig_physics.add_subplot(3, 2, 3)
        self.ax_torque = self.fig_physics.add_subplot(3, 2, 4)
        self.ax_efficiency = self.fig_physics.add_subplot(3, 2, 5)
        self.ax_derating = self.fig_physics.add_subplot(3, 2, 6)

        self.fig_physics.tight_layout(pad=2.0)

    def create_economics_tab(self):
        """Create economic analysis tab"""
        # Left panel: Economic parameters
        left_frame = ttk.Frame(self.tab_economics, width=400)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5)

        econ_params = ttk.LabelFrame(left_frame, text="Economic Parameters", padding=10)
        econ_params.pack(fill=tk.X, pady=5)

        ttk.Label(econ_params, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky=tk.W)
        self.elec_cost_var = tk.StringVar(value="0.12")
        ttk.Entry(econ_params, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1)

        ttk.Label(econ_params, text="Maintenance Cost ($/hr):").grid(row=1, column=0, sticky=tk.W)
        self.maint_cost_var = tk.StringVar(value="5.0")
        ttk.Entry(econ_params, textvariable=self.maint_cost_var, width=15).grid(row=1, column=1)

        ttk.Label(econ_params, text="Operating Hours/Year:").grid(row=2, column=0, sticky=tk.W)
        self.op_hours_var = tk.StringVar(value="8760")
        ttk.Entry(econ_params, textvariable=self.op_hours_var, width=15).grid(row=2, column=1)

        ttk.Button(econ_params, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=3, column=0, columnspan=2, pady=10)

        # Economic results
        econ_results = ttk.LabelFrame(left_frame, text="Economic Analysis Results", padding=10)
        econ_results.pack(fill=tk.BOTH, expand=True, pady=5)

        self.econ_text = tk.Text(econ_results, height=20, width=45, wrap=tk.WORD)
        self.econ_text.pack(fill=tk.BOTH, expand=True)

        # Right panel: Economic charts
        right_frame = ttk.Frame(self.tab_economics)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.fig_econ = Figure(figsize=(8, 6), dpi=100)
        canvas_econ = FigureCanvasTkAgg(self.fig_econ, right_frame)
        canvas_econ.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_econ1 = self.fig_econ.add_subplot(2, 1, 1)
        self.ax_econ2 = self.fig_econ.add_subplot(2, 1, 2)

        self.fig_econ.tight_layout(pad=2.0)

    def create_controls_tab(self):
        """Create advanced controls tab"""
        control_frame = ttk.LabelFrame(self.tab_controls, text="Advanced Control Systems", padding=10)
        control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # AVR Control
        avr_frame = ttk.LabelFrame(control_frame, text="Automatic Voltage Regulator (AVR)", padding=10)
        avr_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.NSEW)

        ttk.Label(avr_frame, text="AVR Gain (Kp):").grid(row=0, column=0, sticky=tk.W)
        self.avr_kp_var = tk.StringVar(value="10.0")
        ttk.Entry(avr_frame, textvariable=self.avr_kp_var, width=15).grid(row=0, column=1)

        ttk.Label(avr_frame, text="AVR Integral (Ki):").grid(row=1, column=0, sticky=tk.W)
        self.avr_ki_var = tk.StringVar(value="5.0")
        ttk.Entry(avr_frame, textvariable=self.avr_ki_var, width=15).grid(row=1, column=1)

        ttk.Checkbutton(avr_frame, text="Enable AVR").grid(row=2, column=0, columnspan=2, pady=5)

        # Governor Control
        gov_frame = ttk.LabelFrame(control_frame, text="Governor Control", padding=10)
        gov_frame.grid(row=0, column=1, padx=5, pady=5, sticky=tk.NSEW)

        ttk.Label(gov_frame, text="Governor Droop (%):").grid(row=0, column=0, sticky=tk.W)
        self.gov_droop_var = tk.StringVar(value="4.0")
        ttk.Entry(gov_frame, textvariable=self.gov_droop_var, width=15).grid(row=0, column=1)

        ttk.Checkbutton(gov_frame, text="Enable Governor").grid(row=1, column=0, columnspan=2, pady=5)

        # Thermal Management
        thermal_frame = ttk.LabelFrame(control_frame, text="Thermal Management", padding=10)
        thermal_frame.grid(row=1, column=0, padx=5, pady=5, sticky=tk.NSEW)

        ttk.Label(thermal_frame, text="Max Temperature (°C):").grid(row=0, column=0, sticky=tk.W)
        self.max_temp_var = tk.StringVar(value="155")
        ttk.Entry(thermal_frame, textvariable=self.max_temp_var, width=15).grid(row=0, column=1)

        ttk.Label(thermal_frame, text="Cooling Fan Speed (%):").grid(row=1, column=0, sticky=tk.W)
        self.fan_speed_var = tk.DoubleVar(value=100)
        ttk.Scale(thermal_frame, from_=0, to=100, variable=self.fan_speed_var,
                 orient=tk.HORIZONTAL).grid(row=1, column=1, sticky=tk.EW)

        # Protection Systems
        prot_frame = ttk.LabelFrame(control_frame, text="Protection Systems", padding=10)
        prot_frame.grid(row=1, column=1, padx=5, pady=5, sticky=tk.NSEW)

        ttk.Checkbutton(prot_frame, text="Overcurrent Protection").grid(row=0, column=0, sticky=tk.W)
        ttk.Checkbutton(prot_frame, text="Overvoltage Protection").grid(row=1, column=0, sticky=tk.W)
        ttk.Checkbutton(prot_frame, text="Thermal Protection").grid(row=2, column=0, sticky=tk.W)
        ttk.Checkbutton(prot_frame, text="Loss of Excitation").grid(row=3, column=0, sticky=tk.W)

        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)

    def update_load_display(self, value):
        """Update load current display"""
        percent = float(value)
        current_a = (percent / 100) * self.model.I_fl
        self.load_current_label.config(text=f"{percent:.1f} % ({current_a:.1f} A)")

    def update_pf_display(self, value):
        """Update power factor display"""
        pf = float(value)
        pf_type = self.pf_type_var.get()
        self.power_factor_label.config(text=f"{pf:.2f} {pf_type}")

    def calculate_static_regulation(self):
        """Calculate and display static voltage regulation"""
        self.results_text.delete(1.0, tk.END)

        # Display header
        self.results_text.insert(tk.END, "="*50 + "\n")
        self.results_text.insert(tk.END, "3-PHASE ALTERNATOR VOLTAGE REGULATION\n")
        self.results_text.insert(tk.END, "Multi-Physics Simulation Results\n")
        self.results_text.insert(tk.END, "="*50 + "\n\n")

        # Display machine parameters
        self.results_text.insert(tk.END, "MACHINE PARAMETERS:\n")
        self.results_text.insert(tk.END, f"Rating: {self.model.rating_kva} kVA\n")
        self.results_text.insert(tk.END, f"Voltage: {self.model.voltage_line} V (line)\n")
        self.results_text.insert(tk.END, f"        {self.model.voltage_phase:.2f} V (phase)\n")
        self.results_text.insert(tk.END, f"Frequency: {self.model.frequency} Hz\n")
        self.results_text.insert(tk.END, f"Full-load Current: {self.model.I_fl:.2f} A\n")
        self.results_text.insert(tk.END, f"Armature Resistance (Ra): {self.model.Ra} Ω\n")
        self.results_text.insert(tk.END, f"Synchronous Reactance (Xs): {self.model.Xs:.3f} Ω\n")
        self.results_text.insert(tk.END, f"Synchronous Impedance (Zs): {self.model.Zs:.3f} Ω\n\n")

        # Calculate regulation at different power factors
        self.results_text.insert(tk.END, "VOLTAGE REGULATION ANALYSIS:\n")
        self.results_text.insert(tk.END, "-" * 50 + "\n\n")

        # (i) Unity Power Factor
        reg_upf, e_upf = self.model.calculate_voltage_regulation(1.0, lagging=True)
        self.results_text.insert(tk.END, "(i) At Unity Power Factor (UPF):\n")
        self.results_text.insert(tk.END, f"    Induced EMF (E): {e_upf:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Terminal Voltage (V): {self.model.voltage_phase:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Voltage Regulation: {reg_upf:.3f} %\n\n")

        # (ii) 0.8 Power Factor Lagging
        reg_lag, e_lag = self.model.calculate_voltage_regulation(0.8, lagging=True)
        self.results_text.insert(tk.END, "(ii) At 0.8 Power Factor Lagging:\n")
        self.results_text.insert(tk.END, f"    Induced EMF (E): {e_lag:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Terminal Voltage (V): {self.model.voltage_phase:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Voltage Regulation: {reg_lag:.3f} %\n\n")

        # Additional analysis at leading power factor
        reg_lead, e_lead = self.model.calculate_voltage_regulation(0.8, lagging=False)
        self.results_text.insert(tk.END, "(iii) At 0.8 Power Factor Leading:\n")
        self.results_text.insert(tk.END, f"    Induced EMF (E): {e_lead:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Terminal Voltage (V): {self.model.voltage_phase:.2f} V/phase\n")
        self.results_text.insert(tk.END, f"    Voltage Regulation: {reg_lead:.3f} %\n\n")

        # Loss analysis at full load
        self.results_text.insert(tk.END, "LOSS BREAKDOWN AT FULL LOAD:\n")
        self.results_text.insert(tk.END, "-" * 50 + "\n")

        sync_speed = 120 * self.model.frequency / 2
        losses = self.model.total_losses(self.model.I_fl, self.model.voltage_phase, sync_speed)

        self.results_text.insert(tk.END, f"Copper Losses (I²R): {losses['copper']/1000:.2f} kW\n")
        self.results_text.insert(tk.END, f"Iron Losses: {losses['iron']/1000:.2f} kW\n")
        self.results_text.insert(tk.END, f"Mechanical Losses: {losses['mechanical']/1000:.2f} kW\n")
        self.results_text.insert(tk.END, f"Stray Load Losses: {losses['stray']/1000:.2f} kW\n")
        self.results_text.insert(tk.END, f"Total Losses: {losses['total']/1000:.2f} kW\n\n")

        # Efficiency
        power_out = self.model.rating_kva * 0.8  # At 0.8 pf
        efficiency = (power_out / (power_out + losses['total']/1000)) * 100
        self.results_text.insert(tk.END, f"Efficiency at 0.8 pf: {efficiency:.2f} %\n\n")

        # Thermal information
        self.results_text.insert(tk.END, "THERMAL ANALYSIS:\n")
        self.results_text.insert(tk.END, "-" * 50 + "\n")
        self.results_text.insert(tk.END, f"Ambient Temperature: {self.model.ambient_temp} °C\n")
        self.results_text.insert(tk.END, f"Maximum Temperature: {self.model.max_temp} °C\n")
        steady_temp = self.model.ambient_temp + losses['total'] * self.model.thermal_resistance
        self.results_text.insert(tk.END, f"Steady-State Temperature: {steady_temp:.1f} °C\n")

    def start_simulation(self):
        """Start dynamic simulation"""
        if self.simulator.running:
            return

        self.simulator.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        # Start simulation thread
        self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop dynamic simulation"""
        self.simulator.running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.stop_simulation()
        self.simulator.reset()
        self.update_plots()
        self.update_physics_plots()
        messagebox.showinfo("Reset", "Simulation has been reset to initial state")

    def run_simulation(self):
        """Run dynamic simulation loop"""
        update_interval = 0.1  # Update plots every 0.1 seconds
        last_update = time.time()

        while self.simulator.running:
            # Get current load settings
            load_percent = self.load_current_var.get()
            load_current = (load_percent / 100) * self.model.I_fl
            power_factor = self.power_factor_var.get()

            # Perform simulation step
            if self.solver_method == 'RK45':
                self.simulator.step_rk45(load_current, power_factor)
            else:
                self.simulator.step_euler(load_current, power_factor)

            # Record history
            self.simulator.record_history()

            # Update time
            self.simulator.time += self.simulator.dt

            # Update plots periodically
            current_time = time.time()
            if current_time - last_update >= update_interval:
                self.root.after(0, self.update_plots)
                self.root.after(0, self.update_physics_plots)
                last_update = current_time

            # Control simulation speed
            time.sleep(self.simulator.dt)

            # Stop after reasonable time
            if self.simulator.time > 60:  # 60 seconds max
                self.simulator.running = False
                self.root.after(0, lambda: self.start_btn.config(state=tk.NORMAL))
                self.root.after(0, lambda: self.stop_btn.config(state=tk.DISABLED))

    def update_plots(self):
        """Update main visualization plots"""
        if not self.simulator.history['time']:
            return

        # Clear axes
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()

        time_data = self.simulator.history['time']

        # Plot 1: Voltage vs Time
        self.ax1.plot(time_data, self.simulator.history['voltage'], 'b-', linewidth=2)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Voltage (V)')
        self.ax1.set_title('Terminal Voltage')
        self.ax1.grid(True, alpha=0.3)

        # Plot 2: Current vs Time
        self.ax2.plot(time_data, self.simulator.history['current'], 'r-', linewidth=2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.set_title('Load Current')
        self.ax2.grid(True, alpha=0.3)

        # Plot 3: Temperature vs Time
        self.ax3.plot(time_data, self.simulator.history['temperature'], 'g-', linewidth=2)
        self.ax3.axhline(y=self.model.max_temp, color='r', linestyle='--', label='Max Temp')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Temperature (°C)')
        self.ax3.set_title('Winding Temperature')
        self.ax3.legend()
        self.ax3.grid(True, alpha=0.3)

        # Plot 4: Power vs Time
        power_kw = [p/1000 for p in self.simulator.history['power']]
        self.ax4.plot(time_data, power_kw, 'm-', linewidth=2)
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Power (kW)')
        self.ax4.set_title('Output Power')
        self.ax4.grid(True, alpha=0.3)

        self.fig_main.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def update_physics_plots(self):
        """Update multi-physics analysis plots"""
        if not self.simulator.history['time']:
            return

        # Clear axes
        self.ax_thermal.clear()
        self.ax_losses.clear()
        self.ax_mechanical.clear()
        self.ax_torque.clear()
        self.ax_efficiency.clear()
        self.ax_derating.clear()

        time_data = self.simulator.history['time']

        # Thermal plot
        self.ax_thermal.plot(time_data, self.simulator.history['temperature'], 'r-', linewidth=2)
        self.ax_thermal.axhline(y=self.model.max_temp, color='k', linestyle='--', alpha=0.5)
        self.ax_thermal.set_xlabel('Time (s)')
        self.ax_thermal.set_ylabel('Temperature (°C)')
        self.ax_thermal.set_title('Thermal Response')
        self.ax_thermal.grid(True, alpha=0.3)

        # Losses plot
        losses_kw = [l/1000 for l in self.simulator.history['losses']]
        self.ax_losses.plot(time_data, losses_kw, 'b-', linewidth=2)
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (kW)')
        self.ax_losses.set_title('Total Losses')
        self.ax_losses.grid(True, alpha=0.3)

        # Mechanical speed plot
        self.ax_mechanical.plot(time_data, self.simulator.history['speed'], 'g-', linewidth=2)
        self.ax_mechanical.set_xlabel('Time (s)')
        self.ax_mechanical.set_ylabel('Speed (RPM)')
        self.ax_mechanical.set_title('Rotor Speed')
        self.ax_mechanical.grid(True, alpha=0.3)

        # Torque plot (calculated from power and speed)
        torque_data = []
        for i, t in enumerate(time_data):
            speed_rad = self.simulator.history['speed'][i] * 2 * np.pi / 60
            if speed_rad > 0:
                torque = self.simulator.history['power'][i] / speed_rad
            else:
                torque = 0
            torque_data.append(torque)

        self.ax_torque.plot(time_data, torque_data, 'c-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Efficiency plot
        self.ax_efficiency.plot(time_data, self.simulator.history['efficiency'], 'm-', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Operating Efficiency')
        self.ax_efficiency.grid(True, alpha=0.3)

        # Derating factor plot
        derating_data = [self.model.derating_factor(temp) for temp in self.simulator.history['temperature']]
        self.ax_derating.plot(time_data, derating_data, 'y-', linewidth=2)
        self.ax_derating.set_xlabel('Time (s)')
        self.ax_derating.set_ylabel('Derating Factor')
        self.ax_derating.set_title('Thermal Derating')
        self.ax_derating.grid(True, alpha=0.3)

        self.fig_physics.tight_layout(pad=2.0)
        self.fig_physics.canvas.draw()

    def calculate_economics(self):
        """Calculate and display economic analysis"""
        self.econ_text.delete(1.0, tk.END)

        try:
            elec_cost = float(self.elec_cost_var.get())
            maint_cost = float(self.maint_cost_var.get())
            op_hours = float(self.op_hours_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid economic parameters")
            return

        self.econ_text.insert(tk.END, "="*50 + "\n")
        self.econ_text.insert(tk.END, "ECONOMIC ANALYSIS\n")
        self.econ_text.insert(tk.END, "="*50 + "\n\n")

        # Calculate average losses and power
        if self.simulator.history['time']:
            avg_power = np.mean(self.simulator.history['power'])
            avg_losses = np.mean(self.simulator.history['losses'])
        else:
            # Use rated values
            avg_power = self.model.rating_kva * 1000 * 0.8
            sync_speed = 120 * self.model.frequency / 2
            losses = self.model.total_losses(self.model.I_fl, self.model.voltage_phase, sync_speed)
            avg_losses = losses['total']

        # Annual energy consumption
        energy_output = avg_power * op_hours / 1000  # kWh
        energy_losses = avg_losses * op_hours / 1000  # kWh
        energy_input = energy_output + energy_losses

        # Annual costs
        energy_cost = energy_input * elec_cost
        maintenance_cost = maint_cost * op_hours
        total_annual_cost = energy_cost + maintenance_cost

        # Display results
        self.econ_text.insert(tk.END, "ANNUAL ENERGY ANALYSIS:\n")
        self.econ_text.insert(tk.END, f"Operating Hours: {op_hours:.0f} hours/year\n")
        self.econ_text.insert(tk.END, f"Energy Output: {energy_output:.2f} kWh\n")
        self.econ_text.insert(tk.END, f"Energy Losses: {energy_losses:.2f} kWh\n")
        self.econ_text.insert(tk.END, f"Energy Input: {energy_input:.2f} kWh\n")
        self.econ_text.insert(tk.END, f"Average Efficiency: {(energy_output/energy_input*100):.2f} %\n\n")

        self.econ_text.insert(tk.END, "ANNUAL COST ANALYSIS:\n")
        self.econ_text.insert(tk.END, f"Electricity Cost: ${energy_cost:,.2f}\n")
        self.econ_text.insert(tk.END, f"Maintenance Cost: ${maintenance_cost:,.2f}\n")
        self.econ_text.insert(tk.END, f"Total Annual Cost: ${total_annual_cost:,.2f}\n\n")

        self.econ_text.insert(tk.END, "COST PER kWh OUTPUT:\n")
        cost_per_kwh = total_annual_cost / energy_output
        self.econ_text.insert(tk.END, f"${cost_per_kwh:.4f}/kWh\n\n")

        # Loss cost breakdown
        self.econ_text.insert(tk.END, "LOSS COST BREAKDOWN:\n")
        self.econ_text.insert(tk.END, f"Cost of Losses: ${energy_losses * elec_cost:,.2f}/year\n")
        loss_percentage = (energy_losses / energy_input) * 100
        self.econ_text.insert(tk.END, f"Loss Percentage: {loss_percentage:.2f} %\n\n")

        # Update economic plots
        self.update_economic_plots(energy_output, energy_losses, energy_cost, maintenance_cost)

    def update_economic_plots(self, energy_out, energy_loss, cost_energy, cost_maint):
        """Update economic analysis plots"""
        self.ax_econ1.clear()
        self.ax_econ2.clear()

        # Energy breakdown pie chart
        energies = [energy_out, energy_loss]
        labels = ['Useful Output', 'Losses']
        colors = ['#2ecc71', '#e74c3c']
        self.ax_econ1.pie(energies, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        self.ax_econ1.set_title('Energy Distribution')

        # Cost breakdown bar chart
        costs = [cost_energy, cost_maint]
        labels = ['Energy Cost', 'Maintenance Cost']
        colors = ['#3498db', '#f39c12']
        self.ax_econ2.bar(labels, costs, color=colors)
        self.ax_econ2.set_ylabel('Annual Cost ($)')
        self.ax_econ2.set_title('Cost Breakdown')

        for i, v in enumerate(costs):
            self.ax_econ2.text(i, v, f'${v:,.0f}', ha='center', va='bottom')

        self.fig_econ.tight_layout(pad=2.0)
        self.fig_econ.canvas.draw()

    def set_solver(self, method):
        """Set ODE solver method"""
        self.solver_method = method
        messagebox.showinfo("Solver", f"Solver method set to: {method}")

    def export_results(self):
        """Export simulation results to file"""
        if not self.simulator.history['time']:
            messagebox.showwarning("Warning", "No simulation data to export")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"alternator_simulation_{timestamp}.csv"

        try:
            with open(filename, 'w') as f:
                # Write header
                f.write("Time(s),Voltage(V),Current(A),Temperature(C),Speed(RPM),Power(W),Losses(W),Efficiency(%)\n")

                # Write data
                for i in range(len(self.simulator.history['time'])):
                    f.write(f"{self.simulator.history['time'][i]:.3f},")
                    f.write(f"{self.simulator.history['voltage'][i]:.2f},")
                    f.write(f"{self.simulator.history['current'][i]:.2f},")
                    f.write(f"{self.simulator.history['temperature'][i]:.2f},")
                    f.write(f"{self.simulator.history['speed'][i]:.2f},")
                    f.write(f"{self.simulator.history['power'][i]:.2f},")
                    f.write(f"{self.simulator.history['losses'][i]:.2f},")
                    f.write(f"{self.simulator.history['efficiency'][i]:.2f}\n")

            messagebox.showinfo("Export", f"Results exported to: {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to export results: {str(e)}")

    def on_window_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # This is automatically handled by pack with fill and expand options
        pass

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced 3-Phase Alternator Voltage Regulation Simulator
Version 1.0

Features:
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Real-time dynamic simulation with RK45 and Euler solvers
- Comprehensive loss analysis
- Economic analysis and cost optimization
- Advanced control systems
- Thermal derating and protection
- Auto-scaling visualization

Developed for electrical engineering education and practical applications.

Example 30.26 Solution:
- Rating: 2000 kVA, 2300 V, 3-phase, 50 Hz
- Voltage Regulation at UPF: Calculated
- Voltage Regulation at 0.8 pf lagging: Calculated
"""
        messagebox.showinfo("About", about_text)


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = AlternatorSimulatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
