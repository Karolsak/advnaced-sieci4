"""
Advanced Shunt Motor Speed Control Simulator with Multi-Physics Analysis
Includes electromagnetic-thermal-mechanical coupling and economic analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import threading
import time
from datetime import datetime

class ShuntMotorSimulator:
    """Advanced DC Shunt Motor Simulator with Multi-Physics Modeling"""

    def __init__(self):
        # Motor parameters
        self.V_supply = 240.0  # Supply voltage (V)
        self.Ra = 0.6  # Armature resistance (Ohms)
        self.Rf = 120.0  # Field resistance (Ohms)
        self.La = 0.01  # Armature inductance (H)
        self.Lf = 5.0  # Field inductance (H)
        self.J = 0.05  # Moment of inertia (kg.m^2)
        self.B = 0.01  # Friction coefficient (N.m.s)
        self.R_added = 0.0  # Added series resistance
        self.T_load = 0.0  # Load torque (N.m)

        # Physical constants
        self.k_phi = 1.5  # Flux constant (Wb)
        self.ambient_temp = 25.0  # Ambient temperature (°C)
        self.thermal_resistance = 2.0  # Thermal resistance (°C/W)
        self.thermal_capacitance = 100.0  # Thermal capacitance (J/°C)
        self.max_temp = 155.0  # Maximum temperature (°C)

        # Economic parameters
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost_per_hour = 5.0  # $/hour
        self.motor_efficiency = 0.85

        # Simulation state
        self.simulation_running = False
        self.simulation_time = 0.0
        self.dt = 0.001  # Time step for Euler method

        # State variables: [Ia, If, omega, theta, T_motor]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 25.0])

        # Data storage for plotting
        self.time_data = []
        self.ia_data = []
        self.speed_data = []
        self.torque_data = []
        self.power_data = []
        self.temp_data = []
        self.efficiency_data = []
        self.losses_copper_data = []
        self.losses_iron_data = []
        self.losses_mechanical_data = []

        # Economic tracking
        self.total_energy_consumed = 0.0  # kWh
        self.total_operating_time = 0.0  # hours

    def solve_example_30_25(self):
        """
        Solve Example 30.25:
        240V shunt motor, Ia=15A at 800 rpm, Ra=0.6Ω
        1. Resistance needed to reduce speed to 400 rpm at same torque
        2. Speed if torque is halved with this resistance
        """
        V = 240.0
        Ia1 = 15.0
        N1 = 800.0
        Ra = 0.6

        # Initial back EMF
        Eb1 = V - Ia1 * Ra

        # Question 1: Same torque (same Ia), N2 = 400 rpm
        N2 = 400.0
        Ia2 = Ia1  # Same torque means same Ia
        Eb2 = Eb1 * (N2 / N1)  # Eb proportional to speed

        # V = Eb2 + Ia2 * (Ra + R_added)
        R_added = (V - Eb2) / Ia2 - Ra

        # Question 2: Torque halved means Ia halved
        Ia3 = Ia1 / 2.0
        R_total = Ra + R_added
        Eb3 = V - Ia3 * R_total
        N3 = N1 * (Eb3 / Eb1)

        results = {
            'Eb1': Eb1,
            'R_added': R_added,
            'N2': N2,
            'Eb2': Eb2,
            'Ia3': Ia3,
            'Eb3': Eb3,
            'N3': N3,
            'R_total': R_total
        }

        return results

    def calculate_flux(self, If):
        """Calculate magnetic flux based on field current"""
        # Saturation curve approximation
        phi_max = self.k_phi
        If_sat = 2.0  # Saturation current
        return phi_max * np.tanh(If / If_sat)

    def calculate_iron_losses(self, omega, phi):
        """Calculate iron losses (hysteresis + eddy current)"""
        # Simplified iron loss model
        n = abs(omega) / (2 * np.pi)  # Speed in Hz
        # Hysteresis losses: P_h = k_h * f * B^2
        P_hysteresis = 0.5 * n * phi**2
        # Eddy current losses: P_e = k_e * f^2 * B^2
        P_eddy = 0.02 * (n**2) * phi**2
        return P_hysteresis + P_eddy

    def calculate_mechanical_losses(self, omega):
        """Calculate mechanical friction and windage losses"""
        # Friction losses proportional to speed
        P_friction = self.B * omega**2
        # Windage losses proportional to speed cubed
        P_windage = 0.001 * abs(omega)**3
        return P_friction + P_windage

    def calculate_stray_losses(self, Ia, omega):
        """Calculate stray load losses"""
        # Empirical formula for stray losses
        return 0.01 * (Ia**2 + 0.001 * omega**2)

    def motor_differential_equations(self, t, y, V_applied, R_series, T_load):
        """
        Multi-physics differential equations for shunt motor
        State vector y = [Ia, If, omega, theta, T_motor]
        """
        Ia, If, omega, theta, T_motor = y

        # Calculate flux
        phi = self.calculate_flux(If)

        # Back EMF
        Eb = phi * omega

        # Electrical equations
        # Armature circuit: V = Eb + Ia*(Ra + R_series) + La*dIa/dt
        dIa_dt = (V_applied - Eb - Ia * (self.Ra + R_series)) / self.La

        # Field circuit: V = If*Rf + Lf*dIf/dt
        dIf_dt = (V_applied - If * self.Rf) / self.Lf

        # Electromagnetic torque
        T_em = phi * Ia

        # Mechanical equation: J*dω/dt = T_em - T_load - B*ω
        domega_dt = (T_em - T_load - self.B * omega) / self.J

        # Angle
        dtheta_dt = omega

        # Thermal equation
        # Heat generation
        P_copper_armature = Ia**2 * (self.Ra + R_series)
        P_copper_field = If**2 * self.Rf
        P_iron = self.calculate_iron_losses(omega, phi)
        P_mechanical = self.calculate_mechanical_losses(omega)
        P_stray = self.calculate_stray_losses(Ia, omega)

        P_total_losses = P_copper_armature + P_copper_field + P_iron + P_mechanical + P_stray

        # Thermal dynamics: C*dT/dt = P_losses - (T - T_ambient)/R_th
        dT_dt = (P_total_losses - (T_motor - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dT_dt]

    def euler_step(self, t, y, dt, V_applied, R_series, T_load):
        """Euler method for ODE integration"""
        dy_dt = self.motor_differential_equations(t, y, V_applied, R_series, T_load)
        return y + np.array(dy_dt) * dt

    def rk45_step(self, t, y, dt, V_applied, R_series, T_load):
        """Runge-Kutta 45 method for ODE integration"""
        k1 = np.array(self.motor_differential_equations(t, y, V_applied, R_series, T_load))
        k2 = np.array(self.motor_differential_equations(t + dt/2, y + k1*dt/2, V_applied, R_series, T_load))
        k3 = np.array(self.motor_differential_equations(t + dt/2, y + k2*dt/2, V_applied, R_series, T_load))
        k4 = np.array(self.motor_differential_equations(t + dt, y + k3*dt, V_applied, R_series, T_load))
        return y + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

    def calculate_performance_metrics(self, Ia, If, omega):
        """Calculate efficiency, power, losses"""
        phi = self.calculate_flux(If)

        # Power calculations
        P_input = self.V_supply * (Ia + If)
        P_copper_armature = Ia**2 * (self.Ra + self.R_added)
        P_copper_field = If**2 * self.Rf
        P_iron = self.calculate_iron_losses(omega, phi)
        P_mechanical_loss = self.calculate_mechanical_losses(omega)
        P_stray = self.calculate_stray_losses(Ia, omega)

        P_total_losses = P_copper_armature + P_copper_field + P_iron + P_mechanical_loss + P_stray
        P_output = P_input - P_total_losses

        # Efficiency (avoid division by zero)
        efficiency = (P_output / P_input * 100) if P_input > 0.1 else 0.0

        return {
            'P_input': P_input,
            'P_output': P_output,
            'P_copper_armature': P_copper_armature,
            'P_copper_field': P_copper_field,
            'P_iron': P_iron,
            'P_mechanical': P_mechanical_loss,
            'P_stray': P_stray,
            'P_total_losses': P_total_losses,
            'efficiency': efficiency
        }

    def thermal_derating_factor(self, T_motor):
        """Calculate derating factor based on temperature"""
        if T_motor < 100:
            return 1.0
        elif T_motor < self.max_temp:
            # Linear derating between 100°C and max temp
            return 1.0 - 0.5 * (T_motor - 100) / (self.max_temp - 100)
        else:
            return 0.5  # 50% derating at max temp

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.simulation_time = 0.0
        self.state = np.array([0.0, 0.0, 0.0, 0.0, self.ambient_temp])
        self.time_data.clear()
        self.ia_data.clear()
        self.speed_data.clear()
        self.torque_data.clear()
        self.power_data.clear()
        self.temp_data.clear()
        self.efficiency_data.clear()
        self.losses_copper_data.clear()
        self.losses_iron_data.clear()
        self.losses_mechanical_data.clear()
        self.total_energy_consumed = 0.0
        self.total_operating_time = 0.0


class ShuntMotorGUI:
    """Advanced Tkinter GUI for Shunt Motor Simulator"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Shunt Motor Speed Control Simulator - Multi-Physics Analysis")
        self.root.geometry("1400x900")

        # Initialize simulator
        self.simulator = ShuntMotorSimulator()

        # Simulation control
        self.simulation_thread = None
        self.solver_method = "RK45"  # or "Euler"

        # Configure grid weights for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create main notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_simulation_tab()
        self.create_analysis_tab()
        self.create_economic_tab()
        self.create_thermal_tab()

        # Bind resize event for auto-scaling
        self.root.bind('<Configure>', self.on_window_resize)

        # Update loop for real-time display
        self.update_display()

    def create_main_tab(self):
        """Main control panel and example solution"""
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="Main Control")

        # Configure grid
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text="DC Shunt Motor Speed Control Simulator",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, pady=10)

        # Create left and right panels
        content_frame = ttk.Frame(main_frame)
        content_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        content_frame.grid_columnconfigure(0, weight=1)
        content_frame.grid_columnconfigure(1, weight=1)
        content_frame.grid_rowconfigure(0, weight=1)

        # Left panel - Example 30.25 Solution
        left_panel = ttk.LabelFrame(content_frame, text="Example 30.25 Solution", padding=10)
        left_panel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Solve button
        solve_btn = ttk.Button(left_panel, text="Solve Example 30.25",
                              command=self.solve_example_callback)
        solve_btn.pack(pady=10)

        # Results display
        self.example_results_text = tk.Text(left_panel, height=25, width=50, wrap=tk.WORD)
        self.example_results_text.pack(fill=tk.BOTH, expand=True, pady=5)

        scrollbar = ttk.Scrollbar(left_panel, command=self.example_results_text.yview)
        self.example_results_text.config(yscrollcommand=scrollbar.set)

        # Right panel - Motor Parameters
        right_panel = ttk.LabelFrame(content_frame, text="Motor Parameters", padding=10)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        # Parameter inputs
        params = [
            ("Supply Voltage (V):", "V_supply", 240.0, 0, 500),
            ("Armature Resistance (Ω):", "Ra", 0.6, 0.1, 10),
            ("Field Resistance (Ω):", "Rf", 120.0, 10, 500),
            ("Armature Inductance (H):", "La", 0.01, 0.001, 1),
            ("Field Inductance (H):", "Lf", 5.0, 0.1, 20),
            ("Inertia (kg.m²):", "J", 0.05, 0.01, 1),
            ("Friction Coeff (N.m.s):", "B", 0.01, 0.001, 1),
        ]

        self.param_sliders = {}

        for i, (label, param_name, default, min_val, max_val) in enumerate(params):
            frame = ttk.Frame(right_panel)
            frame.pack(fill=tk.X, pady=5)

            ttk.Label(frame, text=label, width=25).pack(side=tk.LEFT)

            value_label = ttk.Label(frame, text=f"{default:.3f}", width=10)
            value_label.pack(side=tk.RIGHT)

            slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                             command=lambda v, pn=param_name, vl=value_label: self.update_param(pn, v, vl))
            slider.set(default)
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

            self.param_sliders[param_name] = (slider, value_label)

    def create_simulation_tab(self):
        """Real-time simulation and visualization"""
        sim_frame = ttk.Frame(self.notebook)
        self.notebook.add(sim_frame, text="Dynamic Simulation")

        # Configure grid
        sim_frame.grid_rowconfigure(1, weight=1)
        sim_frame.grid_columnconfigure(0, weight=1)

        # Control panel
        control_panel = ttk.LabelFrame(sim_frame, text="Simulation Controls", padding=10)
        control_panel.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        # Control inputs
        control_frame1 = ttk.Frame(control_panel)
        control_frame1.pack(fill=tk.X, pady=5)

        ttk.Label(control_frame1, text="Added Series Resistance (Ω):").pack(side=tk.LEFT, padx=5)
        self.r_added_label = ttk.Label(control_frame1, text="0.00", width=10)
        self.r_added_label.pack(side=tk.RIGHT, padx=5)
        self.r_added_slider = ttk.Scale(control_frame1, from_=0, to=20, orient=tk.HORIZONTAL,
                                       command=lambda v: self.update_control_param('R_added', v))
        self.r_added_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
        self.r_added_slider.set(0)

        control_frame2 = ttk.Frame(control_panel)
        control_frame2.pack(fill=tk.X, pady=5)

        ttk.Label(control_frame2, text="Load Torque (N.m):").pack(side=tk.LEFT, padx=5)
        self.torque_label = ttk.Label(control_frame2, text="0.00", width=10)
        self.torque_label.pack(side=tk.RIGHT, padx=5)
        self.torque_slider = ttk.Scale(control_frame2, from_=0, to=50, orient=tk.HORIZONTAL,
                                      command=lambda v: self.update_control_param('T_load', v))
        self.torque_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
        self.torque_slider.set(0)

        # Solver selection
        solver_frame = ttk.Frame(control_panel)
        solver_frame.pack(fill=tk.X, pady=5)

        ttk.Label(solver_frame, text="ODE Solver:").pack(side=tk.LEFT, padx=5)
        self.solver_var = tk.StringVar(value="RK45")
        ttk.Radiobutton(solver_frame, text="Runge-Kutta 45", variable=self.solver_var,
                       value="RK45").pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(solver_frame, text="Euler", variable=self.solver_var,
                       value="Euler").pack(side=tk.LEFT, padx=10)

        # Buttons
        button_frame = ttk.Frame(control_panel)
        button_frame.pack(fill=tk.X, pady=10)

        self.start_btn = ttk.Button(button_frame, text="Start Simulation",
                                    command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(button_frame, text="Stop Simulation",
                                   command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(button_frame, text="Reset",
                                    command=self.reset_simulation)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # Status
        self.status_label = ttk.Label(control_panel, text="Status: Ready",
                                     font=("Arial", 10, "bold"))
        self.status_label.pack(pady=5)

        # Plots
        plot_frame = ttk.Frame(sim_frame)
        plot_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_rowconfigure(1, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(1, weight=1)

        # Create figures
        self.fig1 = Figure(figsize=(6, 3), dpi=80)
        self.ax1 = self.fig1.add_subplot(111)
        self.ax1.set_title("Speed vs Time")
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Speed (rpm)")
        self.ax1.grid(True)

        self.canvas1 = FigureCanvasTkAgg(self.fig1, plot_frame)
        self.canvas1.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

        self.fig2 = Figure(figsize=(6, 3), dpi=80)
        self.ax2 = self.fig2.add_subplot(111)
        self.ax2.set_title("Armature Current vs Time")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Current (A)")
        self.ax2.grid(True)

        self.canvas2 = FigureCanvasTkAgg(self.fig2, plot_frame)
        self.canvas2.get_tk_widget().grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

        self.fig3 = Figure(figsize=(6, 3), dpi=80)
        self.ax3 = self.fig3.add_subplot(111)
        self.ax3.set_title("Power vs Time")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("Power (W)")
        self.ax3.grid(True)

        self.canvas3 = FigureCanvasTkAgg(self.fig3, plot_frame)
        self.canvas3.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=2, pady=2)

        self.fig4 = Figure(figsize=(6, 3), dpi=80)
        self.ax4 = self.fig4.add_subplot(111)
        self.ax4.set_title("Efficiency vs Time")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("Efficiency (%)")
        self.ax4.grid(True)

        self.canvas4 = FigureCanvasTkAgg(self.fig4, plot_frame)
        self.canvas4.get_tk_widget().grid(row=1, column=1, sticky="nsew", padx=2, pady=2)

    def create_analysis_tab(self):
        """Loss analysis and multi-physics visualization"""
        analysis_frame = ttk.Frame(self.notebook)
        self.notebook.add(analysis_frame, text="Loss Analysis")

        # Configure grid
        analysis_frame.grid_rowconfigure(0, weight=1)
        analysis_frame.grid_columnconfigure(0, weight=1)
        analysis_frame.grid_columnconfigure(1, weight=1)

        # Left panel - Loss breakdown
        left_panel = ttk.LabelFrame(analysis_frame, text="Detailed Loss Breakdown", padding=10)
        left_panel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        left_panel.grid_rowconfigure(0, weight=1)
        left_panel.grid_columnconfigure(0, weight=1)

        # Loss pie chart
        self.fig_losses = Figure(figsize=(6, 6), dpi=80)
        self.ax_losses = self.fig_losses.add_subplot(111)
        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, left_panel)
        self.canvas_losses.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Right panel - Multiple plots
        right_panel = ttk.Frame(analysis_frame)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        right_panel.grid_rowconfigure(0, weight=1)
        right_panel.grid_rowconfigure(1, weight=1)
        right_panel.grid_columnconfigure(0, weight=1)

        # Torque plot
        torque_panel = ttk.LabelFrame(right_panel, text="Torque Analysis", padding=5)
        torque_panel.grid(row=0, column=0, sticky="nsew", pady=5)
        torque_panel.grid_rowconfigure(0, weight=1)
        torque_panel.grid_columnconfigure(0, weight=1)

        self.fig_torque = Figure(figsize=(6, 3), dpi=80)
        self.ax_torque = self.fig_torque.add_subplot(111)
        self.ax_torque.set_title("Electromagnetic Torque vs Time")
        self.ax_torque.set_xlabel("Time (s)")
        self.ax_torque.set_ylabel("Torque (N.m)")
        self.ax_torque.grid(True)

        self.canvas_torque = FigureCanvasTkAgg(self.fig_torque, torque_panel)
        self.canvas_torque.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Loss comparison plot
        loss_panel = ttk.LabelFrame(right_panel, text="Loss Components Over Time", padding=5)
        loss_panel.grid(row=1, column=0, sticky="nsew", pady=5)
        loss_panel.grid_rowconfigure(0, weight=1)
        loss_panel.grid_columnconfigure(0, weight=1)

        self.fig_loss_time = Figure(figsize=(6, 3), dpi=80)
        self.ax_loss_time = self.fig_loss_time.add_subplot(111)
        self.ax_loss_time.set_title("Loss Components")
        self.ax_loss_time.set_xlabel("Time (s)")
        self.ax_loss_time.set_ylabel("Power Loss (W)")
        self.ax_loss_time.grid(True)

        self.canvas_loss_time = FigureCanvasTkAgg(self.fig_loss_time, loss_panel)
        self.canvas_loss_time.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def create_economic_tab(self):
        """Economic analysis and cost calculation"""
        econ_frame = ttk.Frame(self.notebook)
        self.notebook.add(econ_frame, text="Economic Analysis")

        # Configure grid
        econ_frame.grid_rowconfigure(1, weight=1)
        econ_frame.grid_columnconfigure(0, weight=1)

        # Parameters
        param_frame = ttk.LabelFrame(econ_frame, text="Economic Parameters", padding=10)
        param_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        params = [
            ("Electricity Cost ($/kWh):", "electricity_cost", 0.12, 0.05, 0.5),
            ("Maintenance Cost ($/hour):", "maintenance_cost_per_hour", 5.0, 0, 20),
        ]

        for i, (label, param_name, default, min_val, max_val) in enumerate(params):
            frame = ttk.Frame(param_frame)
            frame.pack(fill=tk.X, pady=5)

            ttk.Label(frame, text=label, width=30).pack(side=tk.LEFT)

            value_label = ttk.Label(frame, text=f"{default:.3f}", width=10)
            value_label.pack(side=tk.RIGHT)

            slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                             command=lambda v, pn=param_name, vl=value_label: self.update_econ_param(pn, v, vl))
            slider.set(default)
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # Results display
        results_frame = ttk.LabelFrame(econ_frame, text="Operating Costs", padding=10)
        results_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        results_frame.grid_rowconfigure(0, weight=1)
        results_frame.grid_columnconfigure(0, weight=1)

        # Create text widget for results
        self.econ_results_text = tk.Text(results_frame, height=15, width=60, wrap=tk.WORD,
                                        font=("Courier", 10))
        self.econ_results_text.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        econ_scrollbar = ttk.Scrollbar(results_frame, command=self.econ_results_text.yview)
        econ_scrollbar.grid(row=0, column=1, sticky="ns")
        self.econ_results_text.config(yscrollcommand=econ_scrollbar.set)

        # Cost over time plot
        plot_frame = ttk.Frame(results_frame)
        plot_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)

        self.fig_cost = Figure(figsize=(8, 4), dpi=80)
        self.ax_cost = self.fig_cost.add_subplot(111)
        self.ax_cost.set_title("Cumulative Operating Cost")
        self.ax_cost.set_xlabel("Time (s)")
        self.ax_cost.set_ylabel("Cost ($)")
        self.ax_cost.grid(True)

        self.canvas_cost = FigureCanvasTkAgg(self.fig_cost, plot_frame)
        self.canvas_cost.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def create_thermal_tab(self):
        """Thermal analysis and derating"""
        thermal_frame = ttk.Frame(self.notebook)
        self.notebook.add(thermal_frame, text="Thermal & Derating")

        # Configure grid
        thermal_frame.grid_rowconfigure(1, weight=1)
        thermal_frame.grid_columnconfigure(0, weight=1)

        # Parameters
        param_frame = ttk.LabelFrame(thermal_frame, text="Thermal Parameters", padding=10)
        param_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        params = [
            ("Ambient Temperature (°C):", "ambient_temp", 25.0, 0, 50),
            ("Thermal Resistance (°C/W):", "thermal_resistance", 2.0, 0.5, 10),
            ("Thermal Capacitance (J/°C):", "thermal_capacitance", 100.0, 10, 500),
            ("Max Temperature (°C):", "max_temp", 155.0, 100, 200),
        ]

        for i, (label, param_name, default, min_val, max_val) in enumerate(params):
            frame = ttk.Frame(param_frame)
            frame.pack(fill=tk.X, pady=5)

            ttk.Label(frame, text=label, width=30).pack(side=tk.LEFT)

            value_label = ttk.Label(frame, text=f"{default:.1f}", width=10)
            value_label.pack(side=tk.RIGHT)

            slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                             command=lambda v, pn=param_name, vl=value_label: self.update_thermal_param(pn, v, vl))
            slider.set(default)
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # Plots
        plot_frame = ttk.Frame(thermal_frame)
        plot_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(1, weight=1)

        # Temperature plot
        temp_panel = ttk.LabelFrame(plot_frame, text="Temperature Profile", padding=5)
        temp_panel.grid(row=0, column=0, sticky="nsew", padx=5)
        temp_panel.grid_rowconfigure(0, weight=1)
        temp_panel.grid_columnconfigure(0, weight=1)

        self.fig_temp = Figure(figsize=(6, 5), dpi=80)
        self.ax_temp = self.fig_temp.add_subplot(111)
        self.ax_temp.set_title("Motor Temperature vs Time")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.ax_temp.grid(True)

        self.canvas_temp = FigureCanvasTkAgg(self.fig_temp, temp_panel)
        self.canvas_temp.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Derating info
        derating_panel = ttk.LabelFrame(plot_frame, text="Derating Information", padding=10)
        derating_panel.grid(row=0, column=1, sticky="nsew", padx=5)
        derating_panel.grid_rowconfigure(0, weight=1)
        derating_panel.grid_columnconfigure(0, weight=1)

        self.derating_text = tk.Text(derating_panel, height=20, width=40, wrap=tk.WORD,
                                    font=("Courier", 10))
        self.derating_text.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        derating_scrollbar = ttk.Scrollbar(derating_panel, command=self.derating_text.yview)
        derating_scrollbar.grid(row=0, column=1, sticky="ns")
        self.derating_text.config(yscrollcommand=derating_scrollbar.set)

    def solve_example_callback(self):
        """Solve and display Example 30.25"""
        results = self.simulator.solve_example_30_25()

        output = "=" * 60 + "\n"
        output += "EXAMPLE 30.25 SOLUTION\n"
        output += "=" * 60 + "\n\n"

        output += "PROBLEM STATEMENT:\n"
        output += "-" * 60 + "\n"
        output += "A 240V shunt motor has an armature current of 15A when\n"
        output += "running at 800 rpm against full load torque.\n"
        output += "Armature resistance = 0.6 Ω\n\n"

        output += "QUESTION 1:\n"
        output += "What resistance must be inserted in series to reduce\n"
        output += "speed to 400 rpm at the same torque?\n\n"

        output += "SOLUTION:\n"
        output += "-" * 60 + "\n"
        output += f"Initial Back EMF (Eb1) = V - Ia × Ra\n"
        output += f"                       = 240 - 15 × 0.6\n"
        output += f"                       = {results['Eb1']:.2f} V\n\n"

        output += f"At 400 rpm with same torque (same Ia = 15A):\n"
        output += f"Since Eb ∝ N (speed):\n"
        output += f"Eb2 = Eb1 × (N2/N1)\n"
        output += f"    = {results['Eb1']:.2f} × (400/800)\n"
        output += f"    = {results['Eb2']:.2f} V\n\n"

        output += f"From voltage equation:\n"
        output += f"V = Eb2 + Ia × (Ra + R_added)\n"
        output += f"240 = {results['Eb2']:.2f} + 15 × (0.6 + R_added)\n"
        output += f"R_added = (240 - {results['Eb2']:.2f})/15 - 0.6\n"
        output += f"        = {results['R_added']:.2f} Ω\n\n"

        output += "=" * 60 + "\n"
        output += f"ANSWER 1: R_added = {results['R_added']:.2f} Ω\n"
        output += "=" * 60 + "\n\n"

        output += "QUESTION 2:\n"
        output += "What will be the speed if load torque is halved\n"
        output += "with this resistance in the circuit?\n\n"

        output += "SOLUTION:\n"
        output += "-" * 60 + "\n"
        output += f"Torque halved means Ia halved:\n"
        output += f"Ia3 = 15/2 = {results['Ia3']:.2f} A\n\n"

        output += f"Total resistance = Ra + R_added\n"
        output += f"                 = 0.6 + {results['R_added']:.2f}\n"
        output += f"                 = {results['R_total']:.2f} Ω\n\n"

        output += f"Back EMF:\n"
        output += f"Eb3 = V - Ia3 × R_total\n"
        output += f"    = 240 - {results['Ia3']:.2f} × {results['R_total']:.2f}\n"
        output += f"    = {results['Eb3']:.2f} V\n\n"

        output += f"Speed (since Eb ∝ N):\n"
        output += f"N3 = N1 × (Eb3/Eb1)\n"
        output += f"   = 800 × ({results['Eb3']:.2f}/{results['Eb1']:.2f})\n"
        output += f"   = {results['N3']:.2f} rpm\n\n"

        output += "=" * 60 + "\n"
        output += f"ANSWER 2: Speed = {results['N3']:.2f} rpm\n"
        output += "=" * 60 + "\n\n"

        output += "KEY CONCEPTS:\n"
        output += "-" * 60 + "\n"
        output += "1. For DC shunt motor with constant flux:\n"
        output += "   - Back EMF (Eb) is proportional to speed (N)\n"
        output += "   - Torque (T) is proportional to armature current (Ia)\n"
        output += "2. Voltage equation: V = Eb + Ia × (Ra + R_added)\n"
        output += "3. Adding series resistance reduces speed for same torque\n"
        output += "4. Reducing load torque increases speed\n"

        self.example_results_text.delete(1.0, tk.END)
        self.example_results_text.insert(1.0, output)

    def update_param(self, param_name, value, label):
        """Update motor parameter"""
        val = float(value)
        setattr(self.simulator, param_name, val)
        label.config(text=f"{val:.3f}")

    def update_control_param(self, param_name, value):
        """Update control parameter"""
        val = float(value)
        setattr(self.simulator, param_name, val)
        if param_name == 'R_added':
            self.r_added_label.config(text=f"{val:.2f}")
        elif param_name == 'T_load':
            self.torque_label.config(text=f"{val:.2f}")

    def update_econ_param(self, param_name, value, label):
        """Update economic parameter"""
        val = float(value)
        setattr(self.simulator, param_name, val)
        label.config(text=f"{val:.3f}")

    def update_thermal_param(self, param_name, value, label):
        """Update thermal parameter"""
        val = float(value)
        setattr(self.simulator, param_name, val)
        label.config(text=f"{val:.1f}")

    def start_simulation(self):
        """Start real-time simulation"""
        if not self.simulator.simulation_running:
            self.simulator.simulation_running = True
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Status: Running")

            # Get solver method
            self.solver_method = self.solver_var.get()

            # Start simulation thread
            self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
            self.simulation_thread.start()

    def stop_simulation(self):
        """Stop simulation"""
        self.simulator.simulation_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.simulator.reset_simulation()
        self.clear_plots()
        self.status_label.config(text="Status: Reset")

    def run_simulation(self):
        """Simulation loop running in separate thread"""
        dt = self.simulator.dt

        while self.simulator.simulation_running:
            # Get current control inputs
            V_applied = self.simulator.V_supply
            R_series = self.simulator.R_added
            T_load = self.simulator.T_load

            # Integrate using selected method
            if self.solver_method == "RK45":
                self.simulator.state = self.simulator.rk45_step(
                    self.simulator.simulation_time,
                    self.simulator.state,
                    dt,
                    V_applied,
                    R_series,
                    T_load
                )
            else:  # Euler
                self.simulator.state = self.simulator.euler_step(
                    self.simulator.simulation_time,
                    self.simulator.state,
                    dt,
                    V_applied,
                    R_series,
                    T_load
                )

            # Update time
            self.simulator.simulation_time += dt

            # Extract state
            Ia, If, omega, theta, T_motor = self.simulator.state

            # Convert to RPM
            speed_rpm = omega * 60 / (2 * np.pi)

            # Calculate performance metrics
            metrics = self.simulator.calculate_performance_metrics(Ia, If, omega)

            # Calculate torque
            phi = self.simulator.calculate_flux(If)
            T_em = phi * Ia

            # Store data (every 10 steps to reduce memory)
            if len(self.simulator.time_data) == 0 or \
               self.simulator.simulation_time - self.simulator.time_data[-1] >= 0.01:
                self.simulator.time_data.append(self.simulator.simulation_time)
                self.simulator.ia_data.append(Ia)
                self.simulator.speed_data.append(speed_rpm)
                self.simulator.torque_data.append(T_em)
                self.simulator.power_data.append(metrics['P_input'])
                self.simulator.temp_data.append(T_motor)
                self.simulator.efficiency_data.append(metrics['efficiency'])
                self.simulator.losses_copper_data.append(metrics['P_copper_armature'] + metrics['P_copper_field'])
                self.simulator.losses_iron_data.append(metrics['P_iron'])
                self.simulator.losses_mechanical_data.append(metrics['P_mechanical'])

                # Update economic tracking
                self.simulator.total_energy_consumed += metrics['P_input'] * dt / 3600000  # kWh
                self.simulator.total_operating_time += dt / 3600  # hours

            # Limit data points
            max_points = 5000
            if len(self.simulator.time_data) > max_points:
                self.simulator.time_data = self.simulator.time_data[-max_points:]
                self.simulator.ia_data = self.simulator.ia_data[-max_points:]
                self.simulator.speed_data = self.simulator.speed_data[-max_points:]
                self.simulator.torque_data = self.simulator.torque_data[-max_points:]
                self.simulator.power_data = self.simulator.power_data[-max_points:]
                self.simulator.temp_data = self.simulator.temp_data[-max_points:]
                self.simulator.efficiency_data = self.simulator.efficiency_data[-max_points:]
                self.simulator.losses_copper_data = self.simulator.losses_copper_data[-max_points:]
                self.simulator.losses_iron_data = self.simulator.losses_iron_data[-max_points:]
                self.simulator.losses_mechanical_data = self.simulator.losses_mechanical_data[-max_points:]

            # Sleep to control simulation speed
            time.sleep(dt / 2)  # Run faster than real-time

    def update_display(self):
        """Update all displays and plots"""
        if len(self.simulator.time_data) > 0:
            # Update simulation plots
            self.update_simulation_plots()

            # Update analysis plots
            self.update_analysis_plots()

            # Update economic display
            self.update_economic_display()

            # Update thermal display
            self.update_thermal_display()

        # Schedule next update
        self.root.after(100, self.update_display)

    def update_simulation_plots(self):
        """Update real-time simulation plots"""
        t = self.simulator.time_data

        # Speed plot
        self.ax1.clear()
        self.ax1.plot(t, self.simulator.speed_data, 'b-', linewidth=1.5)
        self.ax1.set_title("Speed vs Time")
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Speed (rpm)")
        self.ax1.grid(True, alpha=0.3)
        self.canvas1.draw()

        # Current plot
        self.ax2.clear()
        self.ax2.plot(t, self.simulator.ia_data, 'r-', linewidth=1.5)
        self.ax2.set_title("Armature Current vs Time")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Current (A)")
        self.ax2.grid(True, alpha=0.3)
        self.canvas2.draw()

        # Power plot
        self.ax3.clear()
        self.ax3.plot(t, self.simulator.power_data, 'g-', linewidth=1.5)
        self.ax3.set_title("Input Power vs Time")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("Power (W)")
        self.ax3.grid(True, alpha=0.3)
        self.canvas3.draw()

        # Efficiency plot
        self.ax4.clear()
        self.ax4.plot(t, self.simulator.efficiency_data, 'm-', linewidth=1.5)
        self.ax4.set_title("Efficiency vs Time")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("Efficiency (%)")
        self.ax4.set_ylim(0, 100)
        self.ax4.grid(True, alpha=0.3)
        self.canvas4.draw()

    def update_analysis_plots(self):
        """Update loss analysis plots"""
        if len(self.simulator.time_data) == 0:
            return

        # Get latest values for pie chart
        if len(self.simulator.losses_copper_data) > 0:
            latest_copper = self.simulator.losses_copper_data[-1]
            latest_iron = self.simulator.losses_iron_data[-1]
            latest_mech = self.simulator.losses_mechanical_data[-1]

            # Pie chart
            self.ax_losses.clear()
            labels = ['Copper Losses', 'Iron Losses', 'Mechanical Losses']
            sizes = [latest_copper, latest_iron, latest_mech]
            colors = ['#ff6b6b', '#4ecdc4', '#45b7d1']

            # Only plot if there are non-zero losses
            if sum(sizes) > 0.01:
                self.ax_losses.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%',
                                  startangle=90)
                self.ax_losses.set_title(f"Loss Distribution\nTotal: {sum(sizes):.2f} W")
            else:
                self.ax_losses.text(0.5, 0.5, 'No losses yet',
                                   ha='center', va='center', transform=self.ax_losses.transAxes)
            self.canvas_losses.draw()

        # Torque plot
        t = self.simulator.time_data
        self.ax_torque.clear()
        self.ax_torque.plot(t, self.simulator.torque_data, 'c-', linewidth=1.5)
        self.ax_torque.set_title("Electromagnetic Torque vs Time")
        self.ax_torque.set_xlabel("Time (s)")
        self.ax_torque.set_ylabel("Torque (N.m)")
        self.ax_torque.grid(True, alpha=0.3)
        self.canvas_torque.draw()

        # Loss components over time
        self.ax_loss_time.clear()
        self.ax_loss_time.plot(t, self.simulator.losses_copper_data, 'r-',
                              linewidth=1.5, label='Copper')
        self.ax_loss_time.plot(t, self.simulator.losses_iron_data, 'b-',
                              linewidth=1.5, label='Iron')
        self.ax_loss_time.plot(t, self.simulator.losses_mechanical_data, 'g-',
                              linewidth=1.5, label='Mechanical')
        self.ax_loss_time.set_title("Loss Components Over Time")
        self.ax_loss_time.set_xlabel("Time (s)")
        self.ax_loss_time.set_ylabel("Power Loss (W)")
        self.ax_loss_time.legend(loc='upper right')
        self.ax_loss_time.grid(True, alpha=0.3)
        self.canvas_loss_time.draw()

    def update_economic_display(self):
        """Update economic analysis"""
        energy_cost = self.simulator.total_energy_consumed * self.simulator.electricity_cost
        maintenance_cost = self.simulator.total_operating_time * self.simulator.maintenance_cost_per_hour
        total_cost = energy_cost + maintenance_cost

        output = "=" * 60 + "\n"
        output += "ECONOMIC ANALYSIS\n"
        output += "=" * 60 + "\n\n"

        output += f"Operating Time:        {self.simulator.total_operating_time:.4f} hours\n"
        output += f"                       ({self.simulator.total_operating_time * 60:.2f} minutes)\n\n"

        output += f"Energy Consumed:       {self.simulator.total_energy_consumed:.6f} kWh\n"
        output += f"                       ({self.simulator.total_energy_consumed * 1000:.4f} Wh)\n\n"

        output += f"Electricity Rate:      ${self.simulator.electricity_cost:.3f} /kWh\n"
        output += f"Energy Cost:           ${energy_cost:.4f}\n\n"

        output += f"Maintenance Rate:      ${self.simulator.maintenance_cost_per_hour:.2f} /hour\n"
        output += f"Maintenance Cost:      ${maintenance_cost:.4f}\n\n"

        output += "-" * 60 + "\n"
        output += f"TOTAL OPERATING COST:  ${total_cost:.4f}\n"
        output += "-" * 60 + "\n\n"

        # Cost breakdown
        if total_cost > 0:
            energy_pct = (energy_cost / total_cost) * 100
            maint_pct = (maintenance_cost / total_cost) * 100
            output += "Cost Breakdown:\n"
            output += f"  Energy:             {energy_pct:.1f}%\n"
            output += f"  Maintenance:        {maint_pct:.1f}%\n\n"

        # Projected costs
        if self.simulator.total_operating_time > 0:
            cost_per_hour = total_cost / self.simulator.total_operating_time
            output += "Projected Costs:\n"
            output += f"  Per Hour:           ${cost_per_hour:.4f}\n"
            output += f"  Per Day (8h):       ${cost_per_hour * 8:.2f}\n"
            output += f"  Per Month (160h):   ${cost_per_hour * 160:.2f}\n"
            output += f"  Per Year (2000h):   ${cost_per_hour * 2000:.2f}\n"

        self.econ_results_text.delete(1.0, tk.END)
        self.econ_results_text.insert(1.0, output)

        # Update cost plot
        if len(self.simulator.time_data) > 0:
            t = np.array(self.simulator.time_data)
            # Calculate cumulative cost
            cum_cost = []
            for i, time_val in enumerate(t):
                hours = time_val / 3600
                if i == 0:
                    energy = 0
                else:
                    energy = self.simulator.total_energy_consumed * (i / len(t))
                cost = energy * self.simulator.electricity_cost + \
                       hours * self.simulator.maintenance_cost_per_hour
                cum_cost.append(cost)

            self.ax_cost.clear()
            self.ax_cost.plot(t, cum_cost, 'b-', linewidth=2)
            self.ax_cost.set_title("Cumulative Operating Cost")
            self.ax_cost.set_xlabel("Time (s)")
            self.ax_cost.set_ylabel("Cost ($)")
            self.ax_cost.grid(True, alpha=0.3)
            self.canvas_cost.draw()

    def update_thermal_display(self):
        """Update thermal analysis"""
        # Temperature plot
        if len(self.simulator.time_data) > 0:
            t = self.simulator.time_data
            self.ax_temp.clear()
            self.ax_temp.plot(t, self.simulator.temp_data, 'r-', linewidth=2, label='Motor Temp')
            self.ax_temp.axhline(y=self.simulator.ambient_temp, color='b', linestyle='--',
                                linewidth=1, label='Ambient')
            self.ax_temp.axhline(y=self.simulator.max_temp, color='r', linestyle='--',
                                linewidth=1, label='Max Temp')
            self.ax_temp.set_title("Motor Temperature vs Time")
            self.ax_temp.set_xlabel("Time (s)")
            self.ax_temp.set_ylabel("Temperature (°C)")
            self.ax_temp.legend(loc='upper right')
            self.ax_temp.grid(True, alpha=0.3)
            self.canvas_temp.draw()

        # Derating information
        if len(self.simulator.temp_data) > 0:
            current_temp = self.simulator.temp_data[-1]
            derating_factor = self.simulator.thermal_derating_factor(current_temp)

            output = "=" * 50 + "\n"
            output += "THERMAL DERATING ANALYSIS\n"
            output += "=" * 50 + "\n\n"

            output += f"Current Temperature:   {current_temp:.2f} °C\n"
            output += f"Ambient Temperature:   {self.simulator.ambient_temp:.2f} °C\n"
            output += f"Temperature Rise:      {current_temp - self.simulator.ambient_temp:.2f} °C\n\n"

            output += f"Maximum Temperature:   {self.simulator.max_temp:.2f} °C\n"
            output += f"Thermal Margin:        {self.simulator.max_temp - current_temp:.2f} °C\n\n"

            output += "-" * 50 + "\n"
            output += f"Derating Factor:       {derating_factor:.2%}\n"
            output += "-" * 50 + "\n\n"

            # Temperature status
            output += "Temperature Status:\n"
            if current_temp < 75:
                output += "  ✓ NORMAL - Operating within safe limits\n"
            elif current_temp < 100:
                output += "  ! WARM - Temperature elevated\n"
            elif current_temp < self.simulator.max_temp:
                output += "  !! HOT - Approaching maximum temperature\n"
                output += f"     Derating to {derating_factor:.0%} capacity\n"
            else:
                output += "  !!! CRITICAL - Maximum temperature exceeded\n"
                output += "     Immediate shutdown recommended\n"

            output += "\nThermal Time Constant:\n"
            tau = self.simulator.thermal_resistance * self.simulator.thermal_capacitance
            output += f"  τ = R_th × C_th = {tau:.2f} seconds\n"
            output += f"                  = {tau/60:.2f} minutes\n\n"

            output += "Cooling Information:\n"
            output += f"  Thermal Resistance:   {self.simulator.thermal_resistance:.2f} °C/W\n"
            output += f"  Thermal Capacitance:  {self.simulator.thermal_capacitance:.2f} J/°C\n"

            self.derating_text.delete(1.0, tk.END)
            self.derating_text.insert(1.0, output)

    def clear_plots(self):
        """Clear all plots"""
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax_torque,
                   self.ax_loss_time, self.ax_temp, self.ax_cost]:
            ax.clear()

        self.ax_losses.clear()

        # Redraw canvases
        for canvas in [self.canvas1, self.canvas2, self.canvas3, self.canvas4,
                      self.canvas_torque, self.canvas_loss_time, self.canvas_losses,
                      self.canvas_temp, self.canvas_cost]:
            canvas.draw()

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Plots will automatically rescale due to grid weights
        pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = ShuntMotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
