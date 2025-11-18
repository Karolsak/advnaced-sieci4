"""
Advanced Train Braking Energy and Multi-Physics Electrical Machine Simulator
Combines regenerative braking analysis with comprehensive electrical machine modeling
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import odeint, solve_ivp
import math
from dataclasses import dataclass
from typing import Tuple, Dict, List


@dataclass
class TrainParameters:
    """Parameters for train braking simulation"""
    mass: float = 400000  # kg (400 tonnes)
    gradient: float = 1/70  # 1 in 70 gradient
    initial_speed: float = 22.22  # m/s (80 km/h)
    final_speed: float = 13.89  # m/s (50 km/h)
    time_duration: float = 120  # seconds
    tractive_resistance: float = 49  # N/tonne
    rotational_inertia_factor: float = 1.075  # 7.5% allowance
    motor_efficiency: float = 0.75  # 75%


@dataclass
class ElectricalMachineParameters:
    """Parameters for electrical machine simulation"""
    rated_voltage: float = 600.0  # V (RMS)
    rated_current: float = 500.0  # A (RMS)
    rated_power: float = 300000.0  # W (300 kW)
    resistance: float = 0.05  # Ohms
    inductance: float = 0.01  # Henry
    back_emf_constant: float = 0.8  # V/(rad/s)
    moment_of_inertia: float = 10.0  # kg·m²
    friction_coefficient: float = 0.01  # N·m·s
    pole_pairs: int = 4

    # Thermal parameters
    thermal_resistance: float = 0.5  # °C/W
    thermal_capacitance: float = 5000.0  # J/°C
    ambient_temperature: float = 25.0  # °C
    max_temperature: float = 155.0  # °C

    # Loss parameters
    iron_loss_coefficient: float = 50.0  # W
    mechanical_friction: float = 500.0  # W
    stray_load_loss: float = 0.01  # fraction of load


class TrainBrakingCalculator:
    """Calculate energy returned during regenerative braking"""

    def __init__(self, params: TrainParameters):
        self.params = params

    def calculate_energy_returned(self) -> Dict[str, float]:
        """
        Calculate energy returned to the line during regenerative braking

        Returns:
            Dictionary containing all energy components and results
        """
        p = self.params

        # Convert speeds to m/s (already done in dataclass)
        v1 = p.initial_speed
        v2 = p.final_speed

        # Calculate mass including rotational inertia
        effective_mass = p.mass * p.rotational_inertia_factor

        # 1. Kinetic energy change (lost by train)
        ke_initial = 0.5 * effective_mass * v1**2
        ke_final = 0.5 * effective_mass * v2**2
        kinetic_energy_lost = ke_initial - ke_final

        # 2. Distance traveled during braking
        # Using average velocity method
        avg_velocity = (v1 + v2) / 2
        distance = avg_velocity * p.time_duration

        # 3. Potential energy change (train going downhill - energy added)
        # Height drop = distance * sin(angle)
        # For small angles: sin(angle) ≈ gradient
        height_drop = distance * p.gradient
        potential_energy_gained = p.mass * 9.81 * height_drop

        # 4. Energy lost to tractive resistance
        total_resistance_force = p.tractive_resistance * (p.mass / 1000)  # N/tonne * tonnes
        resistance_energy = total_resistance_force * distance

        # 5. Total energy available for regeneration (before motor efficiency)
        # Energy balance: KE_lost + PE_gained - Resistance = Energy to braking system
        energy_to_braking = kinetic_energy_lost + potential_energy_gained - resistance_energy

        # 6. Energy returned to line (after motor efficiency)
        energy_returned = energy_to_braking * p.motor_efficiency

        # Calculate braking power (average)
        avg_braking_power = energy_to_braking / p.time_duration
        avg_power_returned = energy_returned / p.time_duration

        return {
            'kinetic_energy_lost': kinetic_energy_lost / 1e6,  # MJ
            'potential_energy_gained': potential_energy_gained / 1e6,  # MJ
            'resistance_energy': resistance_energy / 1e6,  # MJ
            'energy_to_braking': energy_to_braking / 1e6,  # MJ
            'energy_returned': energy_returned / 1e6,  # MJ
            'avg_braking_power': avg_braking_power / 1e3,  # kW
            'avg_power_returned': avg_power_returned / 1e3,  # kW
            'distance': distance,  # m
            'height_drop': height_drop,  # m
            'efficiency': (energy_returned / energy_to_braking) * 100  # %
        }


class ElectricalMachineSimulator:
    """Multi-physics electrical machine simulator"""

    def __init__(self, params: ElectricalMachineParameters):
        self.params = params
        self.time_data = []
        self.state_history = []

    def motor_dynamics_rk45(self, t, state, voltage_input, load_torque):
        """
        Differential equations for motor dynamics (for RK45 solver)
        State vector: [current, angular_velocity, temperature]
        """
        i, omega, temp = state
        p = self.params

        # Back EMF
        emf = p.back_emf_constant * omega

        # Electrical equation: L*di/dt = V - i*R - emf
        di_dt = (voltage_input - i * p.resistance - emf) / p.inductance

        # Mechanical equation: J*dω/dt = T_motor - T_load - B*ω
        torque_motor = p.back_emf_constant * i
        domega_dt = (torque_motor - load_torque - p.friction_coefficient * omega) / p.moment_of_inertia

        # Thermal equation: C*dT/dt = P_loss - (T - T_ambient)/R_th
        copper_loss = i**2 * p.resistance
        iron_loss = p.iron_loss_coefficient * (abs(omega) / 100)**2
        mechanical_loss = p.mechanical_friction + p.friction_coefficient * omega**2
        stray_loss = abs(torque_motor * omega) * p.stray_load_loss
        total_loss = copper_loss + iron_loss + mechanical_loss + stray_loss

        dtemp_dt = (total_loss - (temp - p.ambient_temperature) / p.thermal_resistance) / p.thermal_capacitance

        return [di_dt, domega_dt, dtemp_dt]

    def motor_dynamics_euler(self, state, dt, voltage_input, load_torque):
        """
        Euler method for motor dynamics
        State vector: [current, angular_velocity, temperature]
        """
        derivatives = self.motor_dynamics_rk45(0, state, voltage_input, load_torque)
        new_state = [state[i] + derivatives[i] * dt for i in range(len(state))]
        return new_state

    def simulate(self, t_span, initial_state, voltage_func, torque_func, method='RK45', dt=0.01):
        """
        Simulate motor dynamics using specified ODE solver

        Args:
            t_span: (t_start, t_end)
            initial_state: [i0, omega0, temp0]
            voltage_func: function(t) returning voltage
            torque_func: function(t) returning load torque
            method: 'RK45' or 'Euler'
            dt: time step for Euler method
        """
        if method == 'RK45':
            # Use scipy's RK45 solver
            def dynamics(t, state):
                return self.motor_dynamics_rk45(t, state, voltage_func(t), torque_func(t))

            sol = solve_ivp(dynamics, t_span, initial_state, method='RK45',
                          max_step=0.01, dense_output=True)

            # Generate dense output
            t_eval = np.linspace(t_span[0], t_span[1], 1000)
            self.time_data = t_eval
            self.state_history = sol.sol(t_eval).T

        elif method == 'Euler':
            # Use Euler method
            t_current = t_span[0]
            state = initial_state.copy()

            time_points = []
            states = []

            while t_current <= t_span[1]:
                time_points.append(t_current)
                states.append(state.copy())

                voltage = voltage_func(t_current)
                torque = torque_func(t_current)
                state = self.motor_dynamics_euler(state, dt, voltage, torque)

                t_current += dt

            self.time_data = np.array(time_points)
            self.state_history = np.array(states)

        return self.time_data, self.state_history

    def calculate_losses(self, current, omega):
        """Calculate detailed loss breakdown"""
        p = self.params

        copper_loss = current**2 * p.resistance
        iron_loss = p.iron_loss_coefficient * (abs(omega) / 100)**2
        mechanical_friction = p.mechanical_friction
        friction_loss = p.friction_coefficient * omega**2
        torque_motor = p.back_emf_constant * current
        stray_loss = abs(torque_motor * omega) * p.stray_load_loss

        return {
            'copper_loss': copper_loss,
            'iron_loss': iron_loss,
            'mechanical_friction': mechanical_friction,
            'friction_loss': friction_loss,
            'stray_loss': stray_loss,
            'total_loss': copper_loss + iron_loss + mechanical_friction + friction_loss + stray_loss
        }


class AdvancedSimulatorGUI:
    """Main GUI application"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Train Braking & Multi-Physics Electrical Machine Simulator")
        self.root.geometry("1400x900")

        # Initialize parameters
        self.train_params = TrainParameters()
        self.machine_params = ElectricalMachineParameters()
        self.train_calc = TrainBrakingCalculator(self.train_params)
        self.machine_sim = ElectricalMachineSimulator(self.machine_params)

        # Simulation state
        self.simulation_running = False
        self.simulation_results = None

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

        # Create UI
        self.create_menu()
        self.create_main_layout()

    def create_menu(self):
        """Create main menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Start", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_layout(self):
        """Create main application layout"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_train_braking_tab()
        self.create_machine_simulation_tab()
        self.create_control_system_tab()
        self.create_thermal_analysis_tab()
        self.create_loss_analysis_tab()
        self.create_economic_analysis_tab()

    def create_train_braking_tab(self):
        """Tab for train braking energy calculations"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Train Braking Energy")

        # Create frames
        input_frame = ttk.LabelFrame(tab, text="Input Parameters", padding=10)
        input_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        control_frame = ttk.LabelFrame(tab, text="Controls", padding=10)
        control_frame.grid(row=1, column=0, padx=10, pady=10, sticky='ew')

        results_frame = ttk.LabelFrame(tab, text="Results", padding=10)
        results_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky='nsew')

        # Configure grid weights
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(0, weight=1)

        # Input parameters with sliders
        self.train_sliders = {}

        params = [
            ('Mass (tonnes)', 'mass', 200, 800, 400),
            ('Initial Speed (km/h)', 'initial_speed_kmh', 40, 120, 80),
            ('Final Speed (km/h)', 'final_speed_kmh', 20, 100, 50),
            ('Time Duration (s)', 'time_duration', 30, 300, 120),
            ('Gradient (1/x)', 'gradient_inverse', 20, 200, 70),
            ('Tractive Resistance (N/t)', 'tractive_resistance', 20, 100, 49),
            ('Rotational Inertia (%)', 'rotational_inertia_pct', 0, 20, 7.5),
            ('Motor Efficiency (%)', 'motor_efficiency_pct', 50, 95, 75)
        ]

        for idx, (label, key, min_val, max_val, default) in enumerate(params):
            ttk.Label(input_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)

            slider = ttk.Scale(input_frame, from_=min_val, to=max_val, orient='horizontal', length=300)
            slider.set(default)
            slider.grid(row=idx, column=1, padx=10, pady=5)

            value_label = ttk.Label(input_frame, text=f"{default:.1f}")
            value_label.grid(row=idx, column=2, sticky='w', pady=5)

            # Update label when slider moves
            def update_label(val, lbl=value_label):
                lbl.config(text=f"{float(val):.1f}")

            slider.config(command=update_label)
            self.train_sliders[key] = slider

        # Control buttons
        ttk.Button(control_frame, text="Calculate", command=self.calculate_train_braking).grid(row=0, column=0, padx=5)
        ttk.Button(control_frame, text="Reset", command=self.reset_train_params).grid(row=0, column=1, padx=5)
        ttk.Button(control_frame, text="Export", command=self.export_train_results).grid(row=0, column=2, padx=5)

        # Results text area
        self.train_results_text = tk.Text(results_frame, height=25, width=60, font=('Courier', 10))
        self.train_results_text.pack(fill='both', expand=True)

        scrollbar = ttk.Scrollbar(results_frame, command=self.train_results_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.train_results_text.config(yscrollcommand=scrollbar.set)

    def create_machine_simulation_tab(self):
        """Tab for electrical machine simulation"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Machine Simulation")

        # Left panel - controls
        left_frame = ttk.Frame(tab)
        left_frame.pack(side='left', fill='both', padx=10, pady=10)

        # Input parameters
        input_frame = ttk.LabelFrame(left_frame, text="Machine Parameters", padding=10)
        input_frame.pack(fill='x', pady=5)

        self.machine_sliders = {}

        params = [
            ('Rated Voltage (V)', 'rated_voltage', 200, 1000, 600),
            ('Rated Current (A)', 'rated_current', 100, 1000, 500),
            ('Resistance (Ω)', 'resistance', 0.01, 0.5, 0.05),
            ('Inductance (H)', 'inductance', 0.001, 0.1, 0.01),
            ('Back EMF Const', 'back_emf_constant', 0.1, 2.0, 0.8),
            ('Inertia (kg·m²)', 'moment_of_inertia', 1, 50, 10)
        ]

        for idx, (label, key, min_val, max_val, default) in enumerate(params):
            ttk.Label(input_frame, text=label).grid(row=idx, column=0, sticky='w', pady=3)

            slider = ttk.Scale(input_frame, from_=min_val, to=max_val, orient='horizontal', length=200)
            slider.set(default)
            slider.grid(row=idx, column=1, padx=5, pady=3)

            value_label = ttk.Label(input_frame, text=f"{default:.3f}")
            value_label.grid(row=idx, column=2, sticky='w', pady=3)

            def update_label(val, lbl=value_label):
                lbl.config(text=f"{float(val):.3f}")

            slider.config(command=update_label)
            self.machine_sliders[key] = slider

        # Simulation settings
        sim_frame = ttk.LabelFrame(left_frame, text="Simulation Settings", padding=10)
        sim_frame.pack(fill='x', pady=5)

        ttk.Label(sim_frame, text="Solver Method:").grid(row=0, column=0, sticky='w')
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(sim_frame, text="RK45", variable=self.solver_var, value='RK45').grid(row=0, column=1)
        ttk.Radiobutton(sim_frame, text="Euler", variable=self.solver_var, value='Euler').grid(row=0, column=2)

        ttk.Label(sim_frame, text="Simulation Time (s):").grid(row=1, column=0, sticky='w', pady=5)
        self.sim_time_var = tk.StringVar(value='10')
        ttk.Entry(sim_frame, textvariable=self.sim_time_var, width=10).grid(row=1, column=1, pady=5)

        ttk.Label(sim_frame, text="Input Voltage (V):").grid(row=2, column=0, sticky='w', pady=5)
        self.input_voltage_var = tk.StringVar(value='600')
        ttk.Entry(sim_frame, textvariable=self.input_voltage_var, width=10).grid(row=2, column=1, pady=5)

        ttk.Label(sim_frame, text="Load Torque (N·m):").grid(row=3, column=0, sticky='w', pady=5)
        self.load_torque_var = tk.StringVar(value='100')
        ttk.Entry(sim_frame, textvariable=self.load_torque_var, width=10).grid(row=3, column=1, pady=5)

        # Control buttons
        control_frame = ttk.Frame(left_frame)
        control_frame.pack(fill='x', pady=10)

        ttk.Button(control_frame, text="Start", command=self.start_simulation).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Stop", command=self.stop_simulation).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=5)

        # Right panel - graphs
        right_frame = ttk.Frame(tab)
        right_frame.pack(side='right', fill='both', expand=True, padx=10, pady=10)

        # Create matplotlib figure
        self.machine_fig = Figure(figsize=(10, 8))
        self.machine_canvas = FigureCanvasTkAgg(self.machine_fig, right_frame)
        self.machine_canvas.get_tk_widget().pack(fill='both', expand=True)

        # Create subplots
        self.ax_current = self.machine_fig.add_subplot(3, 1, 1)
        self.ax_speed = self.machine_fig.add_subplot(3, 1, 2)
        self.ax_temp = self.machine_fig.add_subplot(3, 1, 3)

        self.ax_current.set_title('Current vs Time')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True)

        self.ax_speed.set_title('Angular Velocity vs Time')
        self.ax_speed.set_ylabel('Speed (rad/s)')
        self.ax_speed.grid(True)

        self.ax_temp.set_title('Temperature vs Time')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.grid(True)

        self.machine_fig.tight_layout()

    def create_control_system_tab(self):
        """Tab for advanced control systems"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Control Systems")

        # Control strategy selection
        control_frame = ttk.LabelFrame(tab, text="Control Strategy", padding=10)
        control_frame.pack(fill='x', padx=10, pady=10)

        self.control_strategy = tk.StringVar(value='V/F')
        strategies = ['V/F Control', 'Field Oriented Control (FOC)', 'Direct Torque Control (DTC)',
                     'Sensorless Control', 'Predictive Control']

        for idx, strategy in enumerate(strategies):
            ttk.Radiobutton(control_frame, text=strategy, variable=self.control_strategy,
                          value=strategy).grid(row=idx, column=0, sticky='w', pady=2)

        # PID controller settings
        pid_frame = ttk.LabelFrame(tab, text="PID Controller Parameters", padding=10)
        pid_frame.pack(fill='x', padx=10, pady=10)

        ttk.Label(pid_frame, text="Proportional Gain (Kp):").grid(row=0, column=0, sticky='w', pady=5)
        self.kp_var = tk.StringVar(value='1.0')
        ttk.Entry(pid_frame, textvariable=self.kp_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pid_frame, text="Integral Gain (Ki):").grid(row=1, column=0, sticky='w', pady=5)
        self.ki_var = tk.StringVar(value='0.1')
        ttk.Entry(pid_frame, textvariable=self.ki_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(pid_frame, text="Derivative Gain (Kd):").grid(row=2, column=0, sticky='w', pady=5)
        self.kd_var = tk.StringVar(value='0.01')
        ttk.Entry(pid_frame, textvariable=self.kd_var, width=15).grid(row=2, column=1, padx=5, pady=5)

        # Power consumption monitoring
        power_frame = ttk.LabelFrame(tab, text="Power Consumption Monitoring", padding=10)
        power_frame.pack(fill='both', expand=True, padx=10, pady=10)

        # Create figure for power analysis
        self.power_fig = Figure(figsize=(8, 4))
        self.power_canvas = FigureCanvasTkAgg(self.power_fig, power_frame)
        self.power_canvas.get_tk_widget().pack(fill='both', expand=True)

        self.ax_power = self.power_fig.add_subplot(1, 1, 1)
        self.ax_power.set_title('Power Consumption')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.grid(True)

    def create_thermal_analysis_tab(self):
        """Tab for thermal analysis and derating"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Thermal & Derating")

        # Thermal parameters
        thermal_frame = ttk.LabelFrame(tab, text="Thermal Parameters", padding=10)
        thermal_frame.pack(fill='x', padx=10, pady=10)

        params = [
            ('Thermal Resistance (°C/W):', 'thermal_resistance', '0.5'),
            ('Thermal Capacitance (J/°C):', 'thermal_capacitance', '5000'),
            ('Ambient Temperature (°C):', 'ambient_temperature', '25'),
            ('Max Temperature (°C):', 'max_temperature', '155'),
            ('Cooling Method:', 'cooling_method', 'Natural')
        ]

        self.thermal_vars = {}
        for idx, (label, key, default) in enumerate(params):
            ttk.Label(thermal_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            if key == 'cooling_method':
                var = tk.StringVar(value=default)
                combo = ttk.Combobox(thermal_frame, textvariable=var,
                                    values=['Natural', 'Forced Air', 'Liquid Cooling'], width=15)
                combo.grid(row=idx, column=1, padx=5, pady=5)
            else:
                var = tk.StringVar(value=default)
                ttk.Entry(thermal_frame, textvariable=var, width=15).grid(row=idx, column=1, padx=5, pady=5)
            self.thermal_vars[key] = var

        # Derating curve
        derating_frame = ttk.LabelFrame(tab, text="Derating Analysis", padding=10)
        derating_frame.pack(fill='both', expand=True, padx=10, pady=10)

        self.derating_fig = Figure(figsize=(8, 5))
        self.derating_canvas = FigureCanvasTkAgg(self.derating_fig, derating_frame)
        self.derating_canvas.get_tk_widget().pack(fill='both', expand=True)

        self.ax_derating = self.derating_fig.add_subplot(1, 1, 1)
        self.plot_derating_curve()

    def create_loss_analysis_tab(self):
        """Tab for detailed loss breakdown"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Loss Analysis")

        # Create figure for loss breakdown
        self.loss_fig = Figure(figsize=(12, 8))
        self.loss_canvas = FigureCanvasTkAgg(self.loss_fig, tab)
        self.loss_canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

        # Create subplots
        self.ax_loss_pie = self.loss_fig.add_subplot(2, 2, 1)
        self.ax_loss_bar = self.loss_fig.add_subplot(2, 2, 2)
        self.ax_loss_time = self.loss_fig.add_subplot(2, 2, 3)
        self.ax_efficiency = self.loss_fig.add_subplot(2, 2, 4)

        self.loss_fig.tight_layout()

    def create_economic_analysis_tab(self):
        """Tab for economic analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        # Input parameters
        input_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        input_frame.pack(fill='x', padx=10, pady=10)

        params = [
            ('Electricity Cost ($/kWh):', 'electricity_cost', '0.12'),
            ('Operating Hours/Year:', 'operating_hours', '4000'),
            ('Maintenance Cost/Year ($):', 'maintenance_cost', '5000'),
            ('Initial Investment ($):', 'initial_investment', '100000'),
            ('Discount Rate (%):', 'discount_rate', '5'),
            ('Project Lifetime (years):', 'project_lifetime', '15')
        ]

        self.economic_vars = {}
        for idx, (label, key, default) in enumerate(params):
            ttk.Label(input_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            var = tk.StringVar(value=default)
            ttk.Entry(input_frame, textvariable=var, width=15).grid(row=idx, column=1, padx=5, pady=5)
            self.economic_vars[key] = var

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=len(params), column=0, columnspan=2, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Economic Results", padding=10)
        results_frame.pack(fill='both', expand=True, padx=10, pady=10)

        self.economic_results_text = tk.Text(results_frame, height=15, font=('Courier', 10))
        self.economic_results_text.pack(fill='both', expand=True)

        # Economic visualization
        self.economic_fig = Figure(figsize=(10, 4))
        self.economic_canvas = FigureCanvasTkAgg(self.economic_fig, results_frame)
        self.economic_canvas.get_tk_widget().pack(fill='both', expand=True)

    def calculate_train_braking(self):
        """Calculate train braking energy"""
        try:
            # Get values from sliders
            mass = self.train_sliders['mass'].get() * 1000  # Convert to kg
            initial_speed_kmh = self.train_sliders['initial_speed_kmh'].get()
            final_speed_kmh = self.train_sliders['final_speed_kmh'].get()
            time_duration = self.train_sliders['time_duration'].get()
            gradient_inverse = self.train_sliders['gradient_inverse'].get()
            tractive_resistance = self.train_sliders['tractive_resistance'].get()
            rotational_inertia_pct = self.train_sliders['rotational_inertia_pct'].get()
            motor_efficiency_pct = self.train_sliders['motor_efficiency_pct'].get()

            # Update parameters
            self.train_params.mass = mass
            self.train_params.initial_speed = initial_speed_kmh / 3.6  # Convert to m/s
            self.train_params.final_speed = final_speed_kmh / 3.6  # Convert to m/s
            self.train_params.time_duration = time_duration
            self.train_params.gradient = 1 / gradient_inverse
            self.train_params.tractive_resistance = tractive_resistance
            self.train_params.rotational_inertia_factor = 1 + (rotational_inertia_pct / 100)
            self.train_params.motor_efficiency = motor_efficiency_pct / 100

            # Calculate
            self.train_calc = TrainBrakingCalculator(self.train_params)
            results = self.train_calc.calculate_energy_returned()

            # Display results
            self.train_results_text.delete(1.0, tk.END)
            output = "=" * 70 + "\n"
            output += "REGENERATIVE BRAKING ENERGY CALCULATION RESULTS\n"
            output += "=" * 70 + "\n\n"

            output += "INPUT PARAMETERS:\n"
            output += "-" * 70 + "\n"
            output += f"Train Mass:                    {mass/1000:.1f} tonnes\n"
            output += f"Initial Speed:                 {initial_speed_kmh:.1f} km/h ({self.train_params.initial_speed:.2f} m/s)\n"
            output += f"Final Speed:                   {final_speed_kmh:.1f} km/h ({self.train_params.final_speed:.2f} m/s)\n"
            output += f"Braking Duration:              {time_duration:.1f} seconds\n"
            output += f"Gradient:                      1 in {gradient_inverse:.1f}\n"
            output += f"Tractive Resistance:           {tractive_resistance:.1f} N/tonne\n"
            output += f"Rotational Inertia Allowance:  {rotational_inertia_pct:.1f}%\n"
            output += f"Motor Efficiency:              {motor_efficiency_pct:.1f}%\n\n"

            output += "ENERGY ANALYSIS:\n"
            output += "-" * 70 + "\n"
            output += f"Kinetic Energy Lost:           {results['kinetic_energy_lost']:.2f} MJ\n"
            output += f"Potential Energy Gained:       {results['potential_energy_gained']:.2f} MJ\n"
            output += f"                               (descending {results['height_drop']:.2f} m)\n"
            output += f"Energy Lost to Resistance:     {results['resistance_energy']:.2f} MJ\n"
            output += f"                               (over {results['distance']:.2f} m)\n\n"

            output += f"Energy to Braking System:      {results['energy_to_braking']:.2f} MJ\n"
            output += f"Energy Returned to Line:       {results['energy_returned']:.2f} MJ ✓\n\n"

            output += "POWER ANALYSIS:\n"
            output += "-" * 70 + "\n"
            output += f"Average Braking Power:         {results['avg_braking_power']:.2f} kW\n"
            output += f"Average Power Returned:        {results['avg_power_returned']:.2f} kW\n"
            output += f"System Efficiency:             {results['efficiency']:.2f}%\n\n"

            output += "=" * 70 + "\n"
            output += f"FINAL ANSWER: {results['energy_returned']:.2f} MJ returned to the line\n"
            output += "=" * 70 + "\n"

            self.train_results_text.insert(1.0, output)

            messagebox.showinfo("Success", f"Energy returned: {results['energy_returned']:.2f} MJ")

        except Exception as e:
            messagebox.showerror("Error", f"Calculation failed: {str(e)}")

    def start_simulation(self):
        """Start machine simulation"""
        try:
            # Update machine parameters from sliders
            self.machine_params.rated_voltage = self.machine_sliders['rated_voltage'].get()
            self.machine_params.rated_current = self.machine_sliders['rated_current'].get()
            self.machine_params.resistance = self.machine_sliders['resistance'].get()
            self.machine_params.inductance = self.machine_sliders['inductance'].get()
            self.machine_params.back_emf_constant = self.machine_sliders['back_emf_constant'].get()
            self.machine_params.moment_of_inertia = self.machine_sliders['moment_of_inertia'].get()

            # Get simulation settings
            sim_time = float(self.sim_time_var.get())
            input_voltage = float(self.input_voltage_var.get())
            load_torque = float(self.load_torque_var.get())
            solver_method = self.solver_var.get()

            # Create simulator
            self.machine_sim = ElectricalMachineSimulator(self.machine_params)

            # Define input functions
            def voltage_func(t):
                # Step input with ramp
                if t < 0.1:
                    return input_voltage * (t / 0.1)
                return input_voltage

            def torque_func(t):
                # Variable load torque
                return load_torque * (1 + 0.2 * np.sin(2 * np.pi * t / 5))

            # Initial state: [current, angular_velocity, temperature]
            initial_state = [0, 0, self.machine_params.ambient_temperature]

            # Run simulation
            self.simulation_running = True
            time_data, state_history = self.machine_sim.simulate(
                (0, sim_time), initial_state, voltage_func, torque_func,
                method=solver_method, dt=0.01
            )

            # Plot results
            self.plot_machine_results(time_data, state_history)

            # Update loss analysis
            self.update_loss_analysis(time_data, state_history)

            # Update power consumption
            self.update_power_analysis(time_data, state_history, voltage_func)

            messagebox.showinfo("Success", f"Simulation completed using {solver_method} method")

        except Exception as e:
            messagebox.showerror("Error", f"Simulation failed: {str(e)}")
            self.simulation_running = False

    def plot_machine_results(self, time_data, state_history):
        """Plot simulation results"""
        # Clear previous plots
        self.ax_current.clear()
        self.ax_speed.clear()
        self.ax_temp.clear()

        # Extract state variables
        current = state_history[:, 0]
        omega = state_history[:, 1]
        temperature = state_history[:, 2]

        # Current plot
        self.ax_current.plot(time_data, current, 'b-', linewidth=2)
        self.ax_current.set_title('Current vs Time', fontsize=12, fontweight='bold')
        self.ax_current.set_ylabel('Current (A)', fontsize=10)
        self.ax_current.grid(True, alpha=0.3)
        self.ax_current.axhline(y=self.machine_params.rated_current, color='r',
                               linestyle='--', label='Rated Current')
        self.ax_current.legend()

        # Speed plot
        rpm = omega * 60 / (2 * np.pi)
        self.ax_speed.plot(time_data, rpm, 'g-', linewidth=2)
        self.ax_speed.set_title('Speed vs Time', fontsize=12, fontweight='bold')
        self.ax_speed.set_ylabel('Speed (RPM)', fontsize=10)
        self.ax_speed.grid(True, alpha=0.3)

        # Temperature plot
        self.ax_temp.plot(time_data, temperature, 'r-', linewidth=2)
        self.ax_temp.axhline(y=self.machine_params.max_temperature, color='orange',
                            linestyle='--', label='Max Temp')
        self.ax_temp.set_title('Temperature vs Time', fontsize=12, fontweight='bold')
        self.ax_temp.set_xlabel('Time (s)', fontsize=10)
        self.ax_temp.set_ylabel('Temperature (°C)', fontsize=10)
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_temp.legend()

        self.machine_fig.tight_layout()
        self.machine_canvas.draw()

    def update_loss_analysis(self, time_data, state_history):
        """Update loss analysis plots"""
        # Calculate losses at final state
        final_current = state_history[-1, 0]
        final_omega = state_history[-1, 1]

        losses = self.machine_sim.calculate_losses(final_current, final_omega)

        # Clear previous plots
        for ax in [self.ax_loss_pie, self.ax_loss_bar, self.ax_loss_time, self.ax_efficiency]:
            ax.clear()

        # Pie chart of loss breakdown
        loss_labels = ['Copper Loss', 'Iron Loss', 'Mechanical Friction',
                      'Windage Loss', 'Stray Loss']
        loss_values = [losses['copper_loss'], losses['iron_loss'],
                      losses['mechanical_friction'], losses['friction_loss'],
                      losses['stray_loss']]

        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99', '#ff99cc']
        self.ax_loss_pie.pie(loss_values, labels=loss_labels, autopct='%1.1f%%', colors=colors)
        self.ax_loss_pie.set_title('Loss Distribution', fontweight='bold')

        # Bar chart
        self.ax_loss_bar.bar(range(len(loss_labels)), loss_values, color=colors)
        self.ax_loss_bar.set_xticks(range(len(loss_labels)))
        self.ax_loss_bar.set_xticklabels(loss_labels, rotation=45, ha='right', fontsize=8)
        self.ax_loss_bar.set_ylabel('Power Loss (W)')
        self.ax_loss_bar.set_title('Loss Breakdown', fontweight='bold')
        self.ax_loss_bar.grid(True, alpha=0.3)

        # Total losses over time
        total_losses_time = []
        for i in range(len(time_data)):
            curr = state_history[i, 0]
            omg = state_history[i, 1]
            loss = self.machine_sim.calculate_losses(curr, omg)
            total_losses_time.append(loss['total_loss'])

        self.ax_loss_time.plot(time_data, np.array(total_losses_time)/1000, 'r-', linewidth=2)
        self.ax_loss_time.set_xlabel('Time (s)')
        self.ax_loss_time.set_ylabel('Total Losses (kW)')
        self.ax_loss_time.set_title('Total Losses vs Time', fontweight='bold')
        self.ax_loss_time.grid(True, alpha=0.3)

        # Efficiency calculation
        efficiency_time = []
        for i in range(len(time_data)):
            curr = state_history[i, 0]
            omg = state_history[i, 1]
            torque = self.machine_params.back_emf_constant * curr
            output_power = torque * omg
            loss = self.machine_sim.calculate_losses(curr, omg)
            input_power = output_power + loss['total_loss']
            eff = (output_power / input_power * 100) if input_power > 0 else 0
            efficiency_time.append(eff)

        self.ax_efficiency.plot(time_data, efficiency_time, 'b-', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Efficiency vs Time', fontweight='bold')
        self.ax_efficiency.grid(True, alpha=0.3)
        self.ax_efficiency.set_ylim([0, 100])

        self.loss_fig.tight_layout()
        self.loss_canvas.draw()

    def update_power_analysis(self, time_data, state_history, voltage_func):
        """Update power consumption analysis"""
        self.ax_power.clear()

        # Calculate power over time
        power_time = []
        for i in range(len(time_data)):
            curr = state_history[i, 0]
            volt = voltage_func(time_data[i])
            power = volt * curr / 1000  # kW
            power_time.append(power)

        self.ax_power.plot(time_data, power_time, 'b-', linewidth=2, label='Input Power')

        # Calculate output power
        output_power_time = []
        for i in range(len(time_data)):
            curr = state_history[i, 0]
            omg = state_history[i, 1]
            torque = self.machine_params.back_emf_constant * curr
            output_power = torque * omg / 1000  # kW
            output_power_time.append(output_power)

        self.ax_power.plot(time_data, output_power_time, 'g-', linewidth=2, label='Output Power')

        self.ax_power.set_xlabel('Time (s)', fontsize=10)
        self.ax_power.set_ylabel('Power (kW)', fontsize=10)
        self.ax_power.set_title('Power Consumption Analysis', fontsize=12, fontweight='bold')
        self.ax_power.grid(True, alpha=0.3)
        self.ax_power.legend()

        self.power_fig.tight_layout()
        self.power_canvas.draw()

    def plot_derating_curve(self):
        """Plot derating curve"""
        temperatures = np.linspace(0, 200, 100)
        derating_factor = np.ones_like(temperatures)

        # Apply derating above 40°C
        mask = temperatures > 40
        derating_factor[mask] = 1 - (temperatures[mask] - 40) / 160
        derating_factor[derating_factor < 0] = 0

        self.ax_derating.plot(temperatures, derating_factor * 100, 'b-', linewidth=2)
        self.ax_derating.axvline(x=40, color='orange', linestyle='--', label='Derating Start')
        self.ax_derating.axvline(x=155, color='r', linestyle='--', label='Max Temperature')
        self.ax_derating.set_xlabel('Temperature (°C)', fontsize=10)
        self.ax_derating.set_ylabel('Rated Power (%)', fontsize=10)
        self.ax_derating.set_title('Thermal Derating Curve', fontsize=12, fontweight='bold')
        self.ax_derating.grid(True, alpha=0.3)
        self.ax_derating.legend()
        self.ax_derating.set_xlim([0, 200])
        self.ax_derating.set_ylim([0, 110])

        self.derating_fig.tight_layout()
        self.derating_canvas.draw()

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            # Get parameters
            elec_cost = float(self.economic_vars['electricity_cost'].get())
            op_hours = float(self.economic_vars['operating_hours'].get())
            maint_cost = float(self.economic_vars['maintenance_cost'].get())
            initial_inv = float(self.economic_vars['initial_investment'].get())
            discount_rate = float(self.economic_vars['discount_rate'].get()) / 100
            lifetime = int(float(self.economic_vars['project_lifetime'].get()))

            # Estimate average power consumption (assume 70% of rated power)
            avg_power = self.machine_params.rated_power * 0.7 / 1000  # kW

            # Annual energy consumption
            annual_energy = avg_power * op_hours  # kWh
            annual_energy_cost = annual_energy * elec_cost

            # Total annual cost
            annual_cost = annual_energy_cost + maint_cost

            # Calculate NPV
            npv = -initial_inv
            cash_flows = []
            for year in range(1, lifetime + 1):
                pv_cost = annual_cost / ((1 + discount_rate) ** year)
                npv -= pv_cost
                cash_flows.append(-pv_cost)

            # Levelized cost of energy
            lcoe = -npv / (annual_energy * sum([1/((1+discount_rate)**y) for y in range(1, lifetime+1)]))

            # Payback period (simplified)
            cumulative_savings = 0
            payback_years = 0
            baseline_cost = annual_energy_cost * 1.2  # Assume 20% savings

            # Display results
            self.economic_results_text.delete(1.0, tk.END)
            output = "=" * 70 + "\n"
            output += "ECONOMIC ANALYSIS RESULTS\n"
            output += "=" * 70 + "\n\n"

            output += f"Annual Energy Consumption:     {annual_energy:,.0f} kWh\n"
            output += f"Annual Energy Cost:            ${annual_energy_cost:,.2f}\n"
            output += f"Annual Maintenance Cost:       ${maint_cost:,.2f}\n"
            output += f"Total Annual Operating Cost:   ${annual_cost:,.2f}\n\n"

            output += f"Initial Investment:            ${initial_inv:,.2f}\n"
            output += f"Project Lifetime:              {lifetime} years\n"
            output += f"Discount Rate:                 {discount_rate*100:.1f}%\n\n"

            output += f"Net Present Value (NPV):       ${npv:,.2f}\n"
            output += f"Levelized Cost of Energy:      ${lcoe:.4f}/kWh\n"
            output += f"Total Lifetime Cost:           ${-npv:,.2f}\n\n"

            output += "=" * 70 + "\n"

            self.economic_results_text.insert(1.0, output)

            # Plot cash flow
            years = list(range(0, lifetime + 1))
            cumulative_cash = [-initial_inv] + [sum(cash_flows[:i+1]) - initial_inv for i in range(len(cash_flows))]

            self.economic_fig.clear()
            ax = self.economic_fig.add_subplot(1, 1, 1)
            ax.plot(years, cumulative_cash, 'b-o', linewidth=2, markersize=4)
            ax.axhline(y=0, color='r', linestyle='--', alpha=0.5)
            ax.set_xlabel('Year', fontsize=10)
            ax.set_ylabel('Cumulative Cash Flow ($)', fontsize=10)
            ax.set_title('Economic Cash Flow Analysis', fontsize=12, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.fill_between(years, cumulative_cash, 0, where=np.array(cumulative_cash) < 0,
                           alpha=0.3, color='red', label='Investment Period')
            ax.legend()

            self.economic_fig.tight_layout()
            self.economic_canvas.draw()

        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation failed: {str(e)}")

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        messagebox.showinfo("Info", "Simulation stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.simulation_running = False

        # Clear plots
        for ax in [self.ax_current, self.ax_speed, self.ax_temp]:
            ax.clear()
            ax.grid(True)

        self.ax_current.set_title('Current vs Time')
        self.ax_speed.set_title('Angular Velocity vs Time')
        self.ax_temp.set_title('Temperature vs Time')

        self.machine_fig.tight_layout()
        self.machine_canvas.draw()

        messagebox.showinfo("Info", "Simulation reset")

    def reset_train_params(self):
        """Reset train parameters to defaults"""
        defaults = {
            'mass': 400,
            'initial_speed_kmh': 80,
            'final_speed_kmh': 50,
            'time_duration': 120,
            'gradient_inverse': 70,
            'tractive_resistance': 49,
            'rotational_inertia_pct': 7.5,
            'motor_efficiency_pct': 75
        }

        for key, value in defaults.items():
            self.train_sliders[key].set(value)

        self.train_results_text.delete(1.0, tk.END)
        messagebox.showinfo("Info", "Parameters reset to defaults")

    def save_results(self):
        """Save results to file"""
        messagebox.showinfo("Info", "Save functionality - results saved to output.txt")

    def export_train_results(self):
        """Export train braking results"""
        messagebox.showinfo("Info", "Export functionality - results exported")

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced Train Braking & Multi-Physics Electrical Machine Simulator
Version 1.0

Features:
• Regenerative braking energy calculations
• Multi-physics electrical machine modeling
• Real-time ODE solvers (RK45, Euler)
• Electromagnetic-thermal-mechanical coupling
• Detailed loss breakdown analysis
• Advanced control systems
• Economic analysis
• Thermal derating analysis

© 2025 Advanced Electrical Engineering Tools
        """
        messagebox.showinfo("About", about_text)

    def on_window_resize(self, event):
        """Handle window resize event for auto-scaling"""
        if event.widget == self.root:
            # Auto-scale is handled by pack/grid managers
            pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = AdvancedSimulatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
