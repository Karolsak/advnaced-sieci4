"""
Advanced DC Motor Multi-Physics Simulator
==========================================
This comprehensive simulator includes:
- Solutions to Problems 10 and 11
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Real-time ODE solvers (RK45, Euler)
- Advanced Tkinter GUI with tabs
- Dynamic visualization
- Economic analysis
- Thermal derating and power consumption analysis
- Detailed loss breakdown
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

class DCMotorSimulator:
    """Advanced DC Motor Simulator with Multi-Physics Capabilities"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_running = False
        self.simulation_time = 0.0
        self.time_step = 0.01
        self.solver_type = "RK45"

        # Motor parameters
        self.V_supply = tk.DoubleVar(value=230.0)  # Supply voltage (V)
        self.Ra = tk.DoubleVar(value=0.5)  # Armature resistance (Ω)
        self.Rf = tk.DoubleVar(value=76.67)  # Field resistance (Ω)
        self.La = tk.DoubleVar(value=0.05)  # Armature inductance (H)
        self.Lf = tk.DoubleVar(value=5.0)  # Field inductance (H)
        self.J = tk.DoubleVar(value=0.1)  # Moment of inertia (kg·m²)
        self.B = tk.DoubleVar(value=0.01)  # Friction coefficient (N·m·s)
        self.Kb = tk.DoubleVar(value=1.0)  # Back EMF constant (V·s/rad)
        self.Kt = tk.DoubleVar(value=1.0)  # Torque constant (N·m/A)
        self.T_load = tk.DoubleVar(value=0.0)  # Load torque (N·m)

        # Thermal parameters
        self.T_ambient = tk.DoubleVar(value=25.0)  # Ambient temperature (°C)
        self.R_th_aa = tk.DoubleVar(value=2.0)  # Armature thermal resistance (°C/W)
        self.R_th_fa = tk.DoubleVar(value=3.0)  # Field thermal resistance (°C/W)
        self.C_th_a = tk.DoubleVar(value=100.0)  # Armature thermal capacitance (J/°C)
        self.C_th_f = tk.DoubleVar(value=150.0)  # Field thermal capacitance (J/°C)

        # Economic parameters
        self.electricity_cost = tk.DoubleVar(value=0.12)  # $/kWh
        self.maintenance_cost = tk.DoubleVar(value=50.0)  # $/year
        self.motor_cost = tk.DoubleVar(value=5000.0)  # $ initial cost

        # State variables
        self.state = {
            'ia': 0.0,  # Armature current
            'if': 0.0,  # Field current
            'omega': 0.0,  # Angular velocity
            'theta': 0.0,  # Angular position
            'T_armature': 25.0,  # Armature temperature
            'T_field': 25.0,  # Field temperature
        }

        # Data storage for plotting
        self.time_data = []
        self.ia_data = []
        self.if_data = []
        self.omega_data = []
        self.torque_data = []
        self.power_data = []
        self.temp_armature_data = []
        self.temp_field_data = []
        self.efficiency_data = []

        # Setup GUI
        self.setup_gui()

        # Bind window resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def setup_gui(self):
        """Setup the complete GUI with tabs"""
        # Create main notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_problems = ttk.Frame(self.notebook)
        self.tab_simulation = ttk.Frame(self.notebook)
        self.tab_controls = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_visualization = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_problems, text="Problems 10 & 11")
        self.notebook.add(self.tab_simulation, text="Dynamic Simulation")
        self.notebook.add(self.tab_controls, text="Advanced Controls")
        self.notebook.add(self.tab_thermal, text="Thermal Analysis")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_visualization, text="Visualization")

        # Setup each tab
        self.setup_problems_tab()
        self.setup_simulation_tab()
        self.setup_controls_tab()
        self.setup_thermal_tab()
        self.setup_economic_tab()
        self.setup_visualization_tab()

    def setup_problems_tab(self):
        """Setup tab for solving Problems 10 and 11"""
        # Problem 10
        problem10_frame = ttk.LabelFrame(self.tab_problems, text="Problem 10: 230V DC Shunt Motor", padding=10)
        problem10_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Input fields for Problem 10
        ttk.Label(problem10_frame, text="Supply Voltage (V):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=230.0)).grid(row=0, column=1, pady=2)

        ttk.Label(problem10_frame, text="Armature Resistance (Ω):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=0.5)).grid(row=1, column=1, pady=2)

        ttk.Label(problem10_frame, text="Initial Field Resistance (Ω):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=76.67)).grid(row=2, column=1, pady=2)

        ttk.Label(problem10_frame, text="Additional Field Resistance (Ω):").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=38.33)).grid(row=3, column=1, pady=2)

        ttk.Label(problem10_frame, text="No-load Current (A):").grid(row=4, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=13.0)).grid(row=4, column=1, pady=2)

        ttk.Label(problem10_frame, text="No-load Speed (rpm):").grid(row=5, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=1000.0)).grid(row=5, column=1, pady=2)

        ttk.Label(problem10_frame, text="Load Current (A):").grid(row=6, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem10_frame, textvariable=tk.DoubleVar(value=42.0)).grid(row=6, column=1, pady=2)

        ttk.Button(problem10_frame, text="Solve Problem 10", command=self.solve_problem_10).grid(row=7, column=0, columnspan=2, pady=10)

        # Result display for Problem 10
        self.problem10_result = tk.Text(problem10_frame, height=15, width=80)
        self.problem10_result.grid(row=8, column=0, columnspan=3, pady=5, padx=5)

        # Problem 11
        problem11_frame = ttk.LabelFrame(self.tab_problems, text="Problem 11: 250V DC Shunt Motor with Flux Weakening", padding=10)
        problem11_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Input fields for Problem 11
        ttk.Label(problem11_frame, text="Supply Voltage (V):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=250.0)).grid(row=0, column=1, pady=2)

        ttk.Label(problem11_frame, text="Armature Resistance (Ω):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=0.4)).grid(row=1, column=1, pady=2)

        ttk.Label(problem11_frame, text="Initial Speed (rpm):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=1000.0)).grid(row=2, column=1, pady=2)

        ttk.Label(problem11_frame, text="Initial Armature Current (A):").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=25.0)).grid(row=3, column=1, pady=2)

        ttk.Label(problem11_frame, text="New Armature Current (A):").grid(row=4, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=50.0)).grid(row=4, column=1, pady=2)

        ttk.Label(problem11_frame, text="Flux Reduction (%):").grid(row=5, column=0, sticky=tk.W, pady=2)
        ttk.Entry(problem11_frame, textvariable=tk.DoubleVar(value=3.0)).grid(row=5, column=1, pady=2)

        ttk.Button(problem11_frame, text="Solve Problem 11", command=self.solve_problem_11).grid(row=6, column=0, columnspan=2, pady=10)

        # Result display for Problem 11
        self.problem11_result = tk.Text(problem11_frame, height=15, width=80)
        self.problem11_result.grid(row=7, column=0, columnspan=3, pady=5, padx=5)

    def setup_simulation_tab(self):
        """Setup dynamic simulation tab"""
        # Control panel
        control_frame = ttk.LabelFrame(self.tab_simulation, text="Simulation Controls", padding=10)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        # Solver selection
        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=0, sticky=tk.W, pady=2)
        solver_combo = ttk.Combobox(control_frame, values=["RK45", "Euler"], state="readonly")
        solver_combo.set("RK45")
        solver_combo.grid(row=0, column=1, pady=2)
        solver_combo.bind("<<ComboboxSelected>>", lambda e: setattr(self, 'solver_type', solver_combo.get()))

        # Time step
        ttk.Label(control_frame, text="Time Step (s):").grid(row=0, column=2, sticky=tk.W, pady=2, padx=(20, 0))
        time_step_entry = ttk.Entry(control_frame, width=10)
        time_step_entry.insert(0, "0.01")
        time_step_entry.grid(row=0, column=3, pady=2)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=1, column=0, columnspan=4, pady=10)

        self.btn_start = ttk.Button(btn_frame, text="Start", command=self.start_simulation)
        self.btn_start.pack(side=tk.LEFT, padx=5)

        self.btn_stop = ttk.Button(btn_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        self.btn_reset = ttk.Button(btn_frame, text="Reset", command=self.reset_simulation)
        self.btn_reset.pack(side=tk.LEFT, padx=5)

        # Motor parameters
        params_frame = ttk.LabelFrame(self.tab_simulation, text="Motor Parameters", padding=10)
        params_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create scrollable frame
        canvas = tk.Canvas(params_frame)
        scrollbar = ttk.Scrollbar(params_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Electrical parameters
        row = 0
        ttk.Label(scrollable_frame, text="Supply Voltage (V):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.V_supply, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0, to=500, variable=self.V_supply, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Armature Resistance (Ω):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.Ra, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.1, to=5, variable=self.Ra, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Field Resistance (Ω):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.Rf, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=10, to=200, variable=self.Rf, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Armature Inductance (H):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.La, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.01, to=0.5, variable=self.La, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Field Inductance (H):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.Lf, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=1, to=10, variable=self.Lf, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        # Mechanical parameters
        row += 1
        ttk.Label(scrollable_frame, text="Moment of Inertia (kg·m²):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.J, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.01, to=1, variable=self.J, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Friction Coefficient (N·m·s):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.B, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.001, to=0.1, variable=self.B, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Back EMF Constant (V·s/rad):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.Kb, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.1, to=5, variable=self.Kb, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Torque Constant (N·m/A):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.Kt, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0.1, to=5, variable=self.Kt, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        row += 1
        ttk.Label(scrollable_frame, text="Load Torque (N·m):").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(scrollable_frame, textvariable=self.T_load, width=15).grid(row=row, column=1, pady=2)
        ttk.Scale(scrollable_frame, from_=0, to=100, variable=self.T_load, orient=tk.HORIZONTAL, length=200).grid(row=row, column=2, pady=2)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def setup_controls_tab(self):
        """Setup advanced controls tab"""
        # PWM Control
        pwm_frame = ttk.LabelFrame(self.tab_controls, text="PWM Control", padding=10)
        pwm_frame.pack(fill=tk.X, padx=5, pady=5)

        self.pwm_duty = tk.DoubleVar(value=100.0)
        self.pwm_freq = tk.DoubleVar(value=1000.0)

        ttk.Label(pwm_frame, text="Duty Cycle (%):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(pwm_frame, textvariable=self.pwm_duty, width=15).grid(row=0, column=1, pady=2)
        ttk.Scale(pwm_frame, from_=0, to=100, variable=self.pwm_duty, orient=tk.HORIZONTAL, length=300).grid(row=0, column=2, pady=2)

        ttk.Label(pwm_frame, text="Frequency (Hz):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(pwm_frame, textvariable=self.pwm_freq, width=15).grid(row=1, column=1, pady=2)
        ttk.Scale(pwm_frame, from_=100, to=10000, variable=self.pwm_freq, orient=tk.HORIZONTAL, length=300).grid(row=1, column=2, pady=2)

        # Speed Control
        speed_frame = ttk.LabelFrame(self.tab_controls, text="Speed Control", padding=10)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)

        self.target_speed = tk.DoubleVar(value=1000.0)
        self.kp = tk.DoubleVar(value=0.1)
        self.ki = tk.DoubleVar(value=0.01)
        self.kd = tk.DoubleVar(value=0.001)

        ttk.Label(speed_frame, text="Target Speed (rpm):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(speed_frame, textvariable=self.target_speed, width=15).grid(row=0, column=1, pady=2)
        ttk.Scale(speed_frame, from_=0, to=3000, variable=self.target_speed, orient=tk.HORIZONTAL, length=300).grid(row=0, column=2, pady=2)

        ttk.Label(speed_frame, text="Kp (Proportional):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(speed_frame, textvariable=self.kp, width=15).grid(row=1, column=1, pady=2)
        ttk.Scale(speed_frame, from_=0, to=1, variable=self.kp, orient=tk.HORIZONTAL, length=300).grid(row=1, column=2, pady=2)

        ttk.Label(speed_frame, text="Ki (Integral):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(speed_frame, textvariable=self.ki, width=15).grid(row=2, column=1, pady=2)
        ttk.Scale(speed_frame, from_=0, to=0.1, variable=self.ki, orient=tk.HORIZONTAL, length=300).grid(row=2, column=2, pady=2)

        ttk.Label(speed_frame, text="Kd (Derivative):").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Entry(speed_frame, textvariable=self.kd, width=15).grid(row=3, column=1, pady=2)
        ttk.Scale(speed_frame, from_=0, to=0.01, variable=self.kd, orient=tk.HORIZONTAL, length=300).grid(row=3, column=2, pady=2)

        # Power and Efficiency Display
        power_frame = ttk.LabelFrame(self.tab_controls, text="Real-time Power Consumption", padding=10)
        power_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.power_labels = {}
        labels = [
            "Input Power (W):", "Output Power (W):", "Copper Losses (W):",
            "Iron Losses (W):", "Mechanical Losses (W):", "Stray Losses (W):",
            "Total Losses (W):", "Efficiency (%):"
        ]

        for i, label in enumerate(labels):
            ttk.Label(power_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            value_label = ttk.Label(power_frame, text="0.00", font=('Arial', 10, 'bold'))
            value_label.grid(row=i, column=1, sticky=tk.W, pady=2, padx=10)
            self.power_labels[label] = value_label

    def setup_thermal_tab(self):
        """Setup thermal analysis tab"""
        # Thermal parameters
        thermal_params_frame = ttk.LabelFrame(self.tab_thermal, text="Thermal Parameters", padding=10)
        thermal_params_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(thermal_params_frame, text="Ambient Temperature (°C):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(thermal_params_frame, textvariable=self.T_ambient, width=15).grid(row=0, column=1, pady=2)
        ttk.Scale(thermal_params_frame, from_=0, to=50, variable=self.T_ambient, orient=tk.HORIZONTAL, length=300).grid(row=0, column=2, pady=2)

        ttk.Label(thermal_params_frame, text="Armature Thermal Resistance (°C/W):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(thermal_params_frame, textvariable=self.R_th_aa, width=15).grid(row=1, column=1, pady=2)

        ttk.Label(thermal_params_frame, text="Field Thermal Resistance (°C/W):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(thermal_params_frame, textvariable=self.R_th_fa, width=15).grid(row=2, column=1, pady=2)

        ttk.Label(thermal_params_frame, text="Armature Thermal Capacitance (J/°C):").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Entry(thermal_params_frame, textvariable=self.C_th_a, width=15).grid(row=3, column=1, pady=2)

        ttk.Label(thermal_params_frame, text="Field Thermal Capacitance (J/°C):").grid(row=4, column=0, sticky=tk.W, pady=2)
        ttk.Entry(thermal_params_frame, textvariable=self.C_th_f, width=15).grid(row=4, column=1, pady=2)

        # Derating information
        derating_frame = ttk.LabelFrame(self.tab_thermal, text="Thermal Derating", padding=10)
        derating_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.derating_text = tk.Text(derating_frame, height=20, width=80)
        self.derating_text.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)

        ttk.Button(derating_frame, text="Calculate Derating", command=self.calculate_derating).pack(pady=5)

    def setup_economic_tab(self):
        """Setup economic analysis tab"""
        # Economic parameters
        econ_params_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters", padding=10)
        econ_params_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(econ_params_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(econ_params_frame, textvariable=self.electricity_cost, width=15).grid(row=0, column=1, pady=2)

        ttk.Label(econ_params_frame, text="Maintenance Cost ($/year):").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Entry(econ_params_frame, textvariable=self.maintenance_cost, width=15).grid(row=1, column=1, pady=2)

        ttk.Label(econ_params_frame, text="Motor Initial Cost ($):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(econ_params_frame, textvariable=self.motor_cost, width=15).grid(row=2, column=1, pady=2)

        # Operating hours
        self.operating_hours_per_day = tk.DoubleVar(value=8.0)
        ttk.Label(econ_params_frame, text="Operating Hours/Day:").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Entry(econ_params_frame, textvariable=self.operating_hours_per_day, width=15).grid(row=3, column=1, pady=2)

        # Analysis results
        analysis_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results", padding=10)
        analysis_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.economic_text = tk.Text(analysis_frame, height=25, width=80)
        self.economic_text.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)

        ttk.Button(analysis_frame, text="Calculate Economics", command=self.calculate_economics).pack(pady=5)

    def setup_visualization_tab(self):
        """Setup visualization tab with dynamic graphs"""
        # Create figure for plots
        self.fig = Figure(figsize=(12, 8), dpi=100)

        # Create subplots
        self.ax1 = self.fig.add_subplot(3, 3, 1)
        self.ax2 = self.fig.add_subplot(3, 3, 2)
        self.ax3 = self.fig.add_subplot(3, 3, 3)
        self.ax4 = self.fig.add_subplot(3, 3, 4)
        self.ax5 = self.fig.add_subplot(3, 3, 5)
        self.ax6 = self.fig.add_subplot(3, 3, 6)
        self.ax7 = self.fig.add_subplot(3, 3, 7)
        self.ax8 = self.fig.add_subplot(3, 3, 8)
        self.ax9 = self.fig.add_subplot(3, 3, 9)

        self.fig.tight_layout(pad=2.0)

        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.tab_visualization)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def solve_problem_10(self):
        """Solve Problem 10: 230V DC Shunt Motor with Field Resistance Change"""
        self.problem10_result.delete(1.0, tk.END)

        # Given data
        V = 230.0  # Supply voltage (V)
        Ra = 0.5  # Armature resistance (Ω)
        Rf1 = 76.67  # Initial field resistance (76 2/3 Ω)
        Rf_add = 38.33  # Additional field resistance (38 1/3 Ω)
        Rf2 = Rf1 + Rf_add  # New field resistance
        I_nl = 13.0  # No-load line current (A)
        N1 = 1000.0  # No-load speed (rpm)
        I_L2 = 42.0  # Load line current (A)

        result = "=" * 80 + "\n"
        result += "PROBLEM 10: 230V DC SHUNT MOTOR WITH FIELD RESISTANCE CHANGE\n"
        result += "=" * 80 + "\n\n"

        result += "Given Data:\n"
        result += "-" * 80 + "\n"
        result += f"Supply Voltage (V): {V} V\n"
        result += f"Armature Resistance (Ra): {Ra} Ω\n"
        result += f"Initial Field Resistance (Rf1): {Rf1:.2f} Ω (76 2/3 Ω)\n"
        result += f"Additional Field Resistance: {Rf_add:.2f} Ω (38 1/3 Ω)\n"
        result += f"New Field Resistance (Rf2): {Rf2:.2f} Ω (115 Ω)\n"
        result += f"No-load Line Current (I_nl): {I_nl} A\n"
        result += f"No-load Speed (N1): {N1} rpm\n"
        result += f"Load Line Current (I_L2): {I_L2} A\n\n"

        # Step 1: Calculate field currents
        If1 = V / Rf1  # Field current with initial resistance
        If2 = V / Rf2  # Field current with additional resistance

        result += "Step 1: Calculate Field Currents\n"
        result += "-" * 80 + "\n"
        result += f"Initial Field Current (If1) = V / Rf1 = {V} / {Rf1:.2f} = {If1:.4f} A\n"
        result += f"New Field Current (If2) = V / Rf2 = {V} / {Rf2:.2f} = {If2:.4f} A\n\n"

        # Step 2: Calculate armature currents
        Ia1 = I_nl - If1  # No-load armature current
        Ia2 = I_L2 - If2  # Load armature current

        result += "Step 2: Calculate Armature Currents\n"
        result += "-" * 80 + "\n"
        result += f"No-load Armature Current (Ia1) = I_nl - If1 = {I_nl} - {If1:.4f} = {Ia1:.4f} A\n"
        result += f"Load Armature Current (Ia2) = I_L2 - If2 = {I_L2} - {If2:.4f} = {Ia2:.4f} A\n\n"

        # Step 3: Calculate back EMFs
        Eb1 = V - Ia1 * Ra  # No-load back EMF
        Eb2 = V - Ia2 * Ra  # Load back EMF

        result += "Step 3: Calculate Back EMFs\n"
        result += "-" * 80 + "\n"
        result += f"No-load Back EMF (Eb1) = V - Ia1 × Ra = {V} - {Ia1:.4f} × {Ra} = {Eb1:.4f} V\n"
        result += f"Load Back EMF (Eb2) = V - Ia2 × Ra = {V} - {Ia2:.4f} × {Ra} = {Eb2:.4f} V\n\n"

        # Step 4: Calculate new speed using speed equation
        # N ∝ Eb/Φ, and Φ ∝ If (assuming linear magnetic circuit)
        # N2/N1 = (Eb2/Eb1) × (If1/If2)
        N2 = N1 * (Eb2 / Eb1) * (If1 / If2)

        result += "Step 4: Calculate New Speed\n"
        result += "-" * 80 + "\n"
        result += "For a DC shunt motor: N ∝ Eb/Φ, where Φ ∝ If\n"
        result += "Therefore: N2/N1 = (Eb2/Eb1) × (If1/If2)\n"
        result += f"N2 = N1 × (Eb2/Eb1) × (If1/If2)\n"
        result += f"N2 = {N1} × ({Eb2:.4f}/{Eb1:.4f}) × ({If1:.4f}/{If2:.4f})\n"
        result += f"N2 = {N1} × {Eb2/Eb1:.4f} × {If1/If2:.4f}\n"
        result += f"N2 = {N2:.2f} rpm\n\n"

        # Additional analysis
        result += "Additional Analysis:\n"
        result += "-" * 80 + "\n"

        # Power calculations
        P_in1 = V * I_nl
        P_in2 = V * I_L2
        P_out1 = Eb1 * Ia1
        P_out2 = Eb2 * Ia2

        result += f"No-load Input Power: {P_in1:.2f} W\n"
        result += f"Load Input Power: {P_in2:.2f} W\n"
        result += f"No-load Armature Power: {P_out1:.2f} W\n"
        result += f"Load Armature Power: {P_out2:.2f} W\n\n"

        # Losses
        P_cu_a1 = Ia1**2 * Ra
        P_cu_a2 = Ia2**2 * Ra
        P_cu_f1 = If1**2 * Rf1
        P_cu_f2 = If2**2 * Rf2

        result += "Copper Losses:\n"
        result += f"No-load Armature Copper Loss: {P_cu_a1:.2f} W\n"
        result += f"Load Armature Copper Loss: {P_cu_a2:.2f} W\n"
        result += f"Initial Field Copper Loss: {P_cu_f1:.2f} W\n"
        result += f"New Field Copper Loss: {P_cu_f2:.2f} W\n\n"

        result += "=" * 80 + "\n"
        result += f"FINAL ANSWER: New Speed = {N2:.2f} rpm ≈ {round(N2/100)*100} rpm\n"
        result += "=" * 80 + "\n"

        self.problem10_result.insert(1.0, result)

    def solve_problem_11(self):
        """Solve Problem 11: 250V DC Shunt Motor with Flux Weakening"""
        self.problem11_result.delete(1.0, tk.END)

        # Given data
        V = 250.0  # Supply voltage (V)
        Ra = 0.4  # Armature resistance (Ω)
        N1 = 1000.0  # Initial speed (rpm)
        Ia1 = 25.0  # Initial armature current (A)
        Ia2 = 50.0  # New armature current (A)
        flux_reduction = 3.0  # Flux reduction in percentage

        result = "=" * 80 + "\n"
        result += "PROBLEM 11: 250V DC SHUNT MOTOR WITH FLUX WEAKENING\n"
        result += "=" * 80 + "\n\n"

        result += "Given Data:\n"
        result += "-" * 80 + "\n"
        result += f"Supply Voltage (V): {V} V\n"
        result += f"Armature Resistance (Ra): {Ra} Ω\n"
        result += f"Initial Speed (N1): {N1} rpm\n"
        result += f"Initial Armature Current (Ia1): {Ia1} A\n"
        result += f"New Armature Current (Ia2): {Ia2} A\n"
        result += f"Flux Reduction: {flux_reduction}%\n\n"

        # Step 1: Calculate back EMFs
        Eb1 = V - Ia1 * Ra  # Initial back EMF
        Eb2 = V - Ia2 * Ra  # New back EMF

        result += "Step 1: Calculate Back EMFs\n"
        result += "-" * 80 + "\n"
        result += f"Initial Back EMF (Eb1) = V - Ia1 × Ra = {V} - {Ia1} × {Ra} = {Eb1:.2f} V\n"
        result += f"New Back EMF (Eb2) = V - Ia2 × Ra = {V} - {Ia2} × {Ra} = {Eb2:.2f} V\n\n"

        # Step 2: Calculate flux ratio
        # Φ2 = Φ1 × (1 - flux_reduction/100)
        flux_ratio = 1 - (flux_reduction / 100)

        result += "Step 2: Calculate Flux Ratio\n"
        result += "-" * 80 + "\n"
        result += f"Flux reduction: {flux_reduction}%\n"
        result += f"Flux ratio (Φ2/Φ1) = 1 - {flux_reduction}/100 = {flux_ratio:.4f}\n\n"

        # Step 3: Calculate new speed
        # N ∝ Eb/Φ
        # N2/N1 = (Eb2/Eb1) × (Φ1/Φ2)
        # N2/N1 = (Eb2/Eb1) / (Φ2/Φ1)
        N2 = N1 * (Eb2 / Eb1) / flux_ratio

        result += "Step 3: Calculate New Speed\n"
        result += "-" * 80 + "\n"
        result += "For a DC shunt motor: N ∝ Eb/Φ\n"
        result += "Therefore: N2/N1 = (Eb2/Eb1) × (Φ1/Φ2)\n"
        result += f"N2 = N1 × (Eb2/Eb1) / (Φ2/Φ1)\n"
        result += f"N2 = {N1} × ({Eb2:.2f}/{Eb1:.2f}) / {flux_ratio:.4f}\n"
        result += f"N2 = {N1} × {Eb2/Eb1:.4f} / {flux_ratio:.4f}\n"
        result += f"N2 = {N1} × {(Eb2/Eb1)/flux_ratio:.4f}\n"
        result += f"N2 = {N2:.2f} rpm\n\n"

        # Additional analysis
        result += "Additional Analysis:\n"
        result += "-" * 80 + "\n"

        # Power calculations
        P_out1 = Eb1 * Ia1  # Initial armature power
        P_out2 = Eb2 * Ia2  # New armature power
        P_in1 = V * Ia1  # Approximate input power (assuming constant field current)
        P_in2 = V * Ia2

        result += f"Initial Armature Power: {P_out1:.2f} W\n"
        result += f"New Armature Power: {P_out2:.2f} W\n"
        result += f"Power increase: {((P_out2-P_out1)/P_out1)*100:.2f}%\n\n"

        # Copper losses
        P_cu1 = Ia1**2 * Ra
        P_cu2 = Ia2**2 * Ra

        result += "Armature Copper Losses:\n"
        result += f"Initial: {P_cu1:.2f} W\n"
        result += f"New: {P_cu2:.2f} W\n"
        result += f"Loss increase: {P_cu2 - P_cu1:.2f} W\n\n"

        # Torque calculations
        # T ∝ Φ × Ia
        torque_ratio = flux_ratio * (Ia2 / Ia1)
        result += "Torque Analysis:\n"
        result += f"Torque ratio (T2/T1) = (Φ2/Φ1) × (Ia2/Ia1) = {flux_ratio:.4f} × {Ia2/Ia1:.4f} = {torque_ratio:.4f}\n"
        result += f"Torque change: {(torque_ratio-1)*100:.2f}%\n\n"

        result += "=" * 80 + "\n"
        result += f"FINAL ANSWER: New Speed = {N2:.2f} rpm\n"
        result += "=" * 80 + "\n"

        self.problem11_result.insert(1.0, result)

    def motor_equations(self, t, y):
        """
        Differential equations for DC shunt motor with multi-physics coupling
        State vector y = [ia, if, omega, theta, T_armature, T_field]
        """
        ia, if_current, omega, theta, T_arm, T_fld = y

        # Get current parameters
        V = self.V_supply.get() * (self.pwm_duty.get() / 100.0)  # Apply PWM
        Ra = self.Ra.get()
        Rf = self.Rf.get()
        La = self.La.get()
        Lf = self.Lf.get()
        J = self.J.get()
        B = self.B.get()
        Kb = self.Kb.get()
        Kt = self.Kt.get()
        T_load = self.T_load.get()

        # Temperature-dependent resistance (0.4% per °C for copper)
        alpha = 0.004
        Ra_temp = Ra * (1 + alpha * (T_arm - 25))
        Rf_temp = Rf * (1 + alpha * (T_fld - 25))

        # Back EMF
        Eb = Kb * if_current * omega

        # Electrical equations
        dia_dt = (V - Eb - ia * Ra_temp) / La
        dif_dt = (V - if_current * Rf_temp) / Lf

        # Electromagnetic torque
        T_em = Kt * if_current * ia

        # Iron losses (proportional to speed squared and flux)
        P_iron = 0.01 * omega**2 * if_current
        T_iron_loss = P_iron / max(omega, 1e-6)

        # Mechanical equation
        domega_dt = (T_em - T_load - B * omega - T_iron_loss) / J

        # Angular position
        dtheta_dt = omega

        # Thermal equations
        # Heat generation
        P_cu_armature = ia**2 * Ra_temp  # Armature copper loss
        P_cu_field = if_current**2 * Rf_temp  # Field copper loss
        P_mech_loss = B * omega**2  # Mechanical friction loss

        # Thermal model
        R_th_aa = self.R_th_aa.get()
        R_th_fa = self.R_th_fa.get()
        C_th_a = self.C_th_a.get()
        C_th_f = self.C_th_f.get()
        T_amb = self.T_ambient.get()

        # Temperature rise differential equations
        dT_arm_dt = (P_cu_armature + 0.5 * P_iron + P_mech_loss - (T_arm - T_amb) / R_th_aa) / C_th_a
        dT_fld_dt = (P_cu_field + 0.5 * P_iron - (T_fld - T_amb) / R_th_fa) / C_th_f

        return [dia_dt, dif_dt, domega_dt, dtheta_dt, dT_arm_dt, dT_fld_dt]

    def start_simulation(self):
        """Start the dynamic simulation"""
        self.is_running = True
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)

        # Initialize state if needed
        if self.simulation_time == 0:
            self.state = {
                'ia': 0.0,
                'if': 0.0,
                'omega': 0.0,
                'theta': 0.0,
                'T_armature': self.T_ambient.get(),
                'T_field': self.T_ambient.get(),
            }
            self.time_data = []
            self.ia_data = []
            self.if_data = []
            self.omega_data = []
            self.torque_data = []
            self.power_data = []
            self.temp_armature_data = []
            self.temp_field_data = []
            self.efficiency_data = []

        # Run simulation step
        self.simulate_step()

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_running = False
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

    def reset_simulation(self):
        """Reset the simulation"""
        self.is_running = False
        self.simulation_time = 0.0
        self.state = {
            'ia': 0.0,
            'if': 0.0,
            'omega': 0.0,
            'theta': 0.0,
            'T_armature': self.T_ambient.get(),
            'T_field': self.T_ambient.get(),
        }
        self.time_data = []
        self.ia_data = []
        self.if_data = []
        self.omega_data = []
        self.torque_data = []
        self.power_data = []
        self.temp_armature_data = []
        self.temp_field_data = []
        self.efficiency_data = []

        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

        # Clear plots
        self.update_plots()

    def simulate_step(self):
        """Execute one simulation step"""
        if not self.is_running:
            return

        # Current state
        y0 = [
            self.state['ia'],
            self.state['if'],
            self.state['omega'],
            self.state['theta'],
            self.state['T_armature'],
            self.state['T_field']
        ]

        # Time span for this step
        t_span = (self.simulation_time, self.simulation_time + self.time_step)

        # Solve using selected method
        if self.solver_type == "RK45":
            sol = solve_ivp(self.motor_equations, t_span, y0, method='RK45', max_step=self.time_step/10)
            y_new = sol.y[:, -1]
        else:  # Euler method
            dydt = self.motor_equations(self.simulation_time, y0)
            y_new = [y0[i] + dydt[i] * self.time_step for i in range(len(y0))]

        # Update state
        self.state['ia'] = y_new[0]
        self.state['if'] = y_new[1]
        self.state['omega'] = y_new[2]
        self.state['theta'] = y_new[3]
        self.state['T_armature'] = y_new[4]
        self.state['T_field'] = y_new[5]

        # Store data
        self.time_data.append(self.simulation_time)
        self.ia_data.append(self.state['ia'])
        self.if_data.append(self.state['if'])
        self.omega_data.append(self.state['omega'])

        # Calculate torque and power
        Kt = self.Kt.get()
        torque = Kt * self.state['if'] * self.state['ia']
        self.torque_data.append(torque)

        power_out = torque * self.state['omega']
        self.power_data.append(power_out)

        self.temp_armature_data.append(self.state['T_armature'])
        self.temp_field_data.append(self.state['T_field'])

        # Calculate efficiency
        V = self.V_supply.get() * (self.pwm_duty.get() / 100.0)
        power_in = V * (self.state['ia'] + self.state['if'])
        efficiency = (power_out / max(power_in, 1e-6)) * 100 if power_in > 0 else 0
        self.efficiency_data.append(max(0, min(100, efficiency)))

        # Update power display
        self.update_power_display()

        # Update plots every 10 steps
        if len(self.time_data) % 10 == 0:
            self.update_plots()

        # Increment time
        self.simulation_time += self.time_step

        # Schedule next step
        if self.is_running and self.simulation_time < 10.0:  # Run for 10 seconds max
            self.root.after(10, self.simulate_step)
        else:
            self.stop_simulation()

    def update_power_display(self):
        """Update real-time power consumption display"""
        V = self.V_supply.get() * (self.pwm_duty.get() / 100.0)
        ia = self.state['ia']
        if_current = self.state['if']
        omega = self.state['omega']
        Ra = self.Ra.get()
        Rf = self.Rf.get()
        Kt = self.Kt.get()

        # Losses calculation
        P_cu_armature = ia**2 * Ra
        P_cu_field = if_current**2 * Rf
        P_copper_total = P_cu_armature + P_cu_field

        P_iron = 0.01 * omega**2 * if_current
        P_mech = self.B.get() * omega**2
        P_stray = 0.005 * V * ia  # Approximate stray losses

        P_in = V * (ia + if_current)
        P_out = Kt * if_current * ia * omega
        P_loss_total = P_copper_total + P_iron + P_mech + P_stray

        efficiency = (P_out / max(P_in, 1e-6)) * 100 if P_in > 0 else 0

        # Update labels
        self.power_labels["Input Power (W):"].config(text=f"{P_in:.2f}")
        self.power_labels["Output Power (W):"].config(text=f"{P_out:.2f}")
        self.power_labels["Copper Losses (W):"].config(text=f"{P_copper_total:.2f}")
        self.power_labels["Iron Losses (W):"].config(text=f"{P_iron:.2f}")
        self.power_labels["Mechanical Losses (W):"].config(text=f"{P_mech:.2f}")
        self.power_labels["Stray Losses (W):"].config(text=f"{P_stray:.2f}")
        self.power_labels["Total Losses (W):"].config(text=f"{P_loss_total:.2f}")
        self.power_labels["Efficiency (%):"].config(text=f"{max(0, min(100, efficiency)):.2f}")

    def update_plots(self):
        """Update all visualization plots"""
        if len(self.time_data) == 0:
            return

        # Clear all axes
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6, self.ax7, self.ax8, self.ax9]:
            ax.clear()

        # Plot 1: Armature Current
        self.ax1.plot(self.time_data, self.ia_data, 'b-', linewidth=2)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Armature Current (A)')
        self.ax1.set_title('Armature Current vs Time')
        self.ax1.grid(True, alpha=0.3)

        # Plot 2: Field Current
        self.ax2.plot(self.time_data, self.if_data, 'r-', linewidth=2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Field Current (A)')
        self.ax2.set_title('Field Current vs Time')
        self.ax2.grid(True, alpha=0.3)

        # Plot 3: Speed
        speed_rpm = [w * 60 / (2 * np.pi) for w in self.omega_data]
        self.ax3.plot(self.time_data, speed_rpm, 'g-', linewidth=2)
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Speed (rpm)')
        self.ax3.set_title('Speed vs Time')
        self.ax3.grid(True, alpha=0.3)

        # Plot 4: Torque
        self.ax4.plot(self.time_data, self.torque_data, 'm-', linewidth=2)
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Torque (N·m)')
        self.ax4.set_title('Electromagnetic Torque vs Time')
        self.ax4.grid(True, alpha=0.3)

        # Plot 5: Power
        power_kw = [p / 1000 for p in self.power_data]
        self.ax5.plot(self.time_data, power_kw, 'c-', linewidth=2)
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (kW)')
        self.ax5.set_title('Output Power vs Time')
        self.ax5.grid(True, alpha=0.3)

        # Plot 6: Efficiency
        self.ax6.plot(self.time_data, self.efficiency_data, 'y-', linewidth=2)
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Efficiency (%)')
        self.ax6.set_title('Efficiency vs Time')
        self.ax6.set_ylim([0, 100])
        self.ax6.grid(True, alpha=0.3)

        # Plot 7: Armature Temperature
        self.ax7.plot(self.time_data, self.temp_armature_data, 'r-', linewidth=2)
        self.ax7.set_xlabel('Time (s)')
        self.ax7.set_ylabel('Temperature (°C)')
        self.ax7.set_title('Armature Temperature vs Time')
        self.ax7.grid(True, alpha=0.3)

        # Plot 8: Field Temperature
        self.ax8.plot(self.time_data, self.temp_field_data, 'orange', linewidth=2)
        self.ax8.set_xlabel('Time (s)')
        self.ax8.set_ylabel('Temperature (°C)')
        self.ax8.set_title('Field Temperature vs Time')
        self.ax8.grid(True, alpha=0.3)

        # Plot 9: Torque-Speed Characteristic
        if len(speed_rpm) > 0:
            self.ax9.plot(speed_rpm, self.torque_data, 'b-', linewidth=2)
            self.ax9.set_xlabel('Speed (rpm)')
            self.ax9.set_ylabel('Torque (N·m)')
            self.ax9.set_title('Torque-Speed Characteristic')
            self.ax9.grid(True, alpha=0.3)

        self.fig.tight_layout(pad=2.0)
        self.canvas.draw()

    def calculate_derating(self):
        """Calculate thermal derating information"""
        self.derating_text.delete(1.0, tk.END)

        result = "=" * 80 + "\n"
        result += "THERMAL DERATING ANALYSIS\n"
        result += "=" * 80 + "\n\n"

        # Current temperatures
        T_arm = self.state['T_armature']
        T_fld = self.state['T_field']
        T_amb = self.T_ambient.get()

        # Typical temperature limits for DC motors
        T_max_insulation_B = 130.0  # Class B insulation
        T_max_insulation_F = 155.0  # Class F insulation
        T_max_insulation_H = 180.0  # Class H insulation

        result += f"Current Operating Conditions:\n"
        result += f"Armature Temperature: {T_arm:.2f} °C\n"
        result += f"Field Temperature: {T_fld:.2f} °C\n"
        result += f"Ambient Temperature: {T_amb:.2f} °C\n\n"

        result += "Temperature Limits by Insulation Class:\n"
        result += f"Class B: {T_max_insulation_B} °C\n"
        result += f"Class F: {T_max_insulation_F} °C\n"
        result += f"Class H: {T_max_insulation_H} °C\n\n"

        # Derating calculations
        result += "Derating Factors (for Class F insulation):\n"
        result += "-" * 80 + "\n"

        T_rated = 40.0  # Rated ambient temperature
        if T_amb > T_rated:
            derating_ambient = 1.0 - 0.01 * (T_amb - T_rated)
            result += f"Ambient Temperature Derating: {derating_ambient:.4f} ({(1-derating_ambient)*100:.2f}% reduction)\n"
        else:
            derating_ambient = 1.0
            result += f"Ambient Temperature Derating: {derating_ambient:.4f} (No derating required)\n"

        # Altitude derating (if applicable)
        altitude = 1000  # meters (example)
        altitude_rated = 1000  # meters
        if altitude > altitude_rated:
            derating_altitude = 1.0 - 0.01 * ((altitude - altitude_rated) / 100)
            result += f"Altitude Derating (at {altitude}m): {derating_altitude:.4f}\n"
        else:
            derating_altitude = 1.0

        # Total derating
        total_derating = derating_ambient * derating_altitude
        result += f"\nTotal Derating Factor: {total_derating:.4f}\n"
        result += f"Derated Power Capacity: {total_derating * 100:.2f}%\n\n"

        # Temperature margins
        result += "Temperature Margins (Class F):\n"
        result += "-" * 80 + "\n"
        margin_arm = T_max_insulation_F - T_arm
        margin_fld = T_max_insulation_F - T_fld

        result += f"Armature Margin: {margin_arm:.2f} °C\n"
        result += f"Field Margin: {margin_fld:.2f} °C\n\n"

        if margin_arm < 20 or margin_fld < 20:
            result += "WARNING: Temperature margin is low! Consider:\n"
            result += "  - Reducing load\n"
            result += "  - Improving cooling\n"
            result += "  - Reducing ambient temperature\n"
            result += "  - Using higher class insulation\n\n"

        # Lifetime estimation
        result += "Estimated Lifetime Impact:\n"
        result += "-" * 80 + "\n"
        # Arrhenius equation: halving life for every 10°C increase
        T_design = 115.0  # Design temperature for Class F
        if T_arm > T_design:
            life_reduction_factor = 2**((T_arm - T_design) / 10.0)
            result += f"Armature operating above design temperature\n"
            result += f"Estimated life reduction factor: {life_reduction_factor:.2f}x\n"
        else:
            life_extension_factor = 2**((T_design - T_arm) / 10.0)
            result += f"Armature operating below design temperature\n"
            result += f"Estimated life extension factor: {life_extension_factor:.2f}x\n"

        self.derating_text.insert(1.0, result)

    def calculate_economics(self):
        """Calculate economic analysis"""
        self.economic_text.delete(1.0, tk.END)

        result = "=" * 80 + "\n"
        result += "ECONOMIC ANALYSIS\n"
        result += "=" * 80 + "\n\n"

        # Operating parameters
        hours_per_day = self.operating_hours_per_day.get()
        days_per_year = 250  # Working days
        electricity_cost = self.electricity_cost.get()
        maintenance_cost = self.maintenance_cost.get()
        motor_cost = self.motor_cost.get()

        # Calculate average power consumption
        if len(self.power_data) > 0:
            avg_power_output = np.mean(self.power_data)  # Watts
            avg_efficiency = np.mean(self.efficiency_data) / 100  # Convert to fraction
            avg_power_input = avg_power_output / max(avg_efficiency, 0.01)  # Watts
        else:
            avg_power_input = 1000  # Default 1 kW
            avg_efficiency = 0.85

        result += "Operating Parameters:\n"
        result += f"Operating hours per day: {hours_per_day:.1f} hours\n"
        result += f"Operating days per year: {days_per_year} days\n"
        result += f"Annual operating hours: {hours_per_day * days_per_year:.1f} hours\n"
        result += f"Average input power: {avg_power_input/1000:.2f} kW\n"
        result += f"Average efficiency: {avg_efficiency*100:.2f}%\n\n"

        # Energy consumption
        daily_energy = (avg_power_input / 1000) * hours_per_day  # kWh
        annual_energy = daily_energy * days_per_year  # kWh

        result += "Energy Consumption:\n"
        result += f"Daily energy consumption: {daily_energy:.2f} kWh\n"
        result += f"Annual energy consumption: {annual_energy:.2f} kWh\n\n"

        # Costs
        daily_electricity_cost = daily_energy * electricity_cost
        annual_electricity_cost = annual_energy * electricity_cost
        total_annual_cost = annual_electricity_cost + maintenance_cost

        result += "Operating Costs:\n"
        result += f"Electricity cost: ${electricity_cost:.3f}/kWh\n"
        result += f"Daily electricity cost: ${daily_electricity_cost:.2f}\n"
        result += f"Annual electricity cost: ${annual_electricity_cost:.2f}\n"
        result += f"Annual maintenance cost: ${maintenance_cost:.2f}\n"
        result += f"Total annual operating cost: ${total_annual_cost:.2f}\n\n"

        # Lifecycle cost (10 years)
        years = 10
        total_lifecycle_cost = motor_cost + (total_annual_cost * years)

        result += f"Lifecycle Cost Analysis ({years} years):\n"
        result += f"Initial motor cost: ${motor_cost:.2f}\n"
        result += f"Total operating cost ({years} years): ${total_annual_cost * years:.2f}\n"
        result += f"Total lifecycle cost: ${total_lifecycle_cost:.2f}\n\n"

        # Energy savings with efficiency improvement
        result += "Efficiency Improvement Analysis:\n"
        result += "-" * 80 + "\n"

        for eff_improvement in [1, 3, 5]:
            new_efficiency = avg_efficiency * (1 + eff_improvement/100)
            new_efficiency = min(new_efficiency, 0.98)  # Cap at 98%

            new_power_input = avg_power_output / new_efficiency
            energy_saved = (avg_power_input - new_power_input) / 1000 * hours_per_day * days_per_year
            cost_saved = energy_saved * electricity_cost

            result += f"\n{eff_improvement}% efficiency improvement (to {new_efficiency*100:.2f}%):\n"
            result += f"  Annual energy savings: {energy_saved:.2f} kWh\n"
            result += f"  Annual cost savings: ${cost_saved:.2f}\n"
            result += f"  {years}-year savings: ${cost_saved * years:.2f}\n"

        # CO2 emissions (assuming 0.5 kg CO2/kWh - typical grid mix)
        co2_factor = 0.5  # kg CO2/kWh
        annual_co2 = annual_energy * co2_factor

        result += f"\nEnvironmental Impact:\n"
        result += f"Annual CO2 emissions: {annual_co2:.2f} kg\n"
        result += f"Annual CO2 emissions: {annual_co2/1000:.2f} tonnes\n"

        # Return on investment for VFD installation
        vfd_cost = motor_cost * 0.3  # Assume VFD costs 30% of motor cost
        vfd_energy_savings = 0.25  # 25% energy savings typical
        vfd_annual_savings = annual_electricity_cost * vfd_energy_savings
        vfd_payback = vfd_cost / vfd_annual_savings

        result += f"\nVariable Frequency Drive (VFD) Analysis:\n"
        result += f"Estimated VFD cost: ${vfd_cost:.2f}\n"
        result += f"Expected energy savings: {vfd_energy_savings*100:.1f}%\n"
        result += f"Annual cost savings: ${vfd_annual_savings:.2f}\n"
        result += f"Simple payback period: {vfd_payback:.2f} years\n"

        self.economic_text.insert(1.0, result)

    def on_window_resize(self, event):
        """Handle window resize event for autoscaling"""
        # Only handle resize for the main window
        if event.widget == self.root:
            # Update canvas size
            if hasattr(self, 'canvas'):
                self.fig.tight_layout(pad=2.0)
                self.canvas.draw()

def main():
    """Main entry point"""
    root = tk.Tk()
    app = DCMotorSimulator(root)
    root.mainloop()

if __name__ == "__main__":
    main()
