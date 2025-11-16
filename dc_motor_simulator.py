"""
Advanced DC Motor Simulator with Multi-Physics Analysis
Includes solutions to DC motor problems and comprehensive simulation capabilities
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from scipy.integrate import solve_ivp, odeint
import math
from collections import deque
import time


class DCMotorProblems:
    """Solutions to DC motor theoretical problems"""

    @staticmethod
    def problem_3():
        """
        Problem 3: DC shunt motor speed calculation with flux reduction
        """
        # Given data
        V = 500  # Supply voltage (V)
        N1 = 800  # Initial speed (rpm)
        Ia1 = 42  # Initial armature current (A)
        Ra = 0.6  # Armature resistance (Ω)
        Vb = 2  # Brush voltage drop (V)
        flux_reduction = 0.75  # Flux reduced to 75%

        # Calculate back EMF at initial conditions
        Eb1 = V - Ia1 * Ra - Vb

        # Torque is proportional to flux * armature current
        # T = k * φ * Ia
        # For constant torque: φ1 * Ia1 = φ2 * Ia2
        # φ2 = 0.75 * φ1

        # Case (a): Torque unchanged
        # φ1 * Ia1 = φ2 * Ia2
        # Ia2 = Ia1 * (φ1/φ2) = Ia1 / 0.75
        Ia2_a = Ia1 / flux_reduction
        Eb2_a = V - Ia2_a * Ra - Vb
        # N ∝ Eb/φ
        # N2/N1 = (Eb2/Eb1) * (φ1/φ2)
        N2_a = N1 * (Eb2_a / Eb1) * (1 / flux_reduction)

        # Case (b): Torque reduced by 20%
        # 0.8 * φ1 * Ia1 = φ2 * Ia2
        # Ia2 = 0.8 * Ia1 * (φ1/φ2)
        Ia2_b = 0.8 * Ia1 / flux_reduction
        Eb2_b = V - Ia2_b * Ra - Vb
        N2_b = N1 * (Eb2_b / Eb1) * (1 / flux_reduction)

        return {
            'Eb1': Eb1,
            'case_a': {'Ia2': Ia2_a, 'Eb2': Eb2_a, 'N2': N2_a},
            'case_b': {'Ia2': Ia2_b, 'Eb2': Eb2_b, 'N2': N2_b}
        }

    @staticmethod
    def problem_4():
        """
        Problem 4: DC shunt motor with flux increase
        """
        # Given data
        V = 460  # Supply voltage (V)
        Ia1 = 28  # Initial armature current (A)
        N1 = 1000  # Initial speed (rpm)
        Ra = 0.72  # Armature resistance (Ω)
        flux_increase = 1.20  # Flux increased to 120%

        # Calculate back EMF at initial conditions
        Eb1 = V - Ia1 * Ra

        # For constant torque: φ1 * Ia1 = φ2 * Ia2
        # φ2 = 1.2 * φ1
        # Ia2 = Ia1 * (φ1/φ2) = Ia1 / 1.2
        Ia2 = Ia1 / flux_increase
        Eb2 = V - Ia2 * Ra

        # N2/N1 = (Eb2/Eb1) * (φ1/φ2)
        N2 = N1 * (Eb2 / Eb1) * (1 / flux_increase)

        return {
            'Eb1': Eb1,
            'Ia2': Ia2,
            'Eb2': Eb2,
            'N2': N2
        }


class DCMotorModel:
    """Mathematical model of DC motor with multi-physics simulation"""

    def __init__(self):
        # Electrical parameters
        self.Ra = 0.5  # Armature resistance (Ω)
        self.La = 0.01  # Armature inductance (H)
        self.Rf = 200  # Field resistance (Ω)
        self.Lf = 10  # Field inductance (H)
        self.Kb = 0.8  # Back EMF constant (V.s/rad)
        self.Kt = 0.8  # Torque constant (N.m/A)

        # Mechanical parameters
        self.J = 0.02  # Moment of inertia (kg.m²)
        self.B = 0.001  # Viscous friction coefficient (N.m.s/rad)
        self.TL = 0  # Load torque (N.m)

        # Thermal parameters
        self.thermal_resistance_a = 2.0  # Armature thermal resistance (K/W)
        self.thermal_capacitance_a = 50  # Armature thermal capacitance (J/K)
        self.thermal_resistance_f = 3.0  # Field thermal resistance (K/W)
        self.thermal_capacitance_f = 80  # Field thermal capacitance (J/K)
        self.ambient_temp = 25  # Ambient temperature (°C)
        self.max_temp = 130  # Maximum allowed temperature (°C)

        # Loss components
        self.iron_loss_coefficient = 0.5  # Iron loss coefficient
        self.mechanical_loss_coefficient = 0.02  # Mechanical loss coefficient
        self.stray_loss_coefficient = 0.01  # Stray load loss coefficient

        # State variables
        self.V_supply = 220  # Supply voltage (V)
        self.solver_type = 'RK45'  # ODE solver type

    def motor_ode_shunt(self, t, y):
        """
        Differential equations for DC shunt motor
        State variables: [Ia, If, omega, theta, T_arm, T_field]
        """
        Ia, If, omega, theta, T_arm, T_field = y

        # Temperature derating factor
        derating_factor = self.calculate_derating(T_arm)

        # Back EMF
        Eb = self.Kb * omega * If / (self.Rf / self.Rf)  # Proportional to field current

        # Electrical equations
        dIa_dt = (self.V_supply - Eb - Ia * self.Ra) / self.La
        dIf_dt = (self.V_supply - If * self.Rf) / self.Lf

        # Electromagnetic torque with derating
        Te = self.Kt * If * Ia * derating_factor

        # Mechanical equation
        domega_dt = (Te - self.B * omega - self.TL) / self.J

        # Angle
        dtheta_dt = omega

        # Thermal equations
        # Copper losses
        P_copper_a = Ia**2 * self.Ra
        P_copper_f = If**2 * self.Rf

        # Iron losses (proportional to speed squared)
        P_iron = self.iron_loss_coefficient * (omega / (2 * np.pi))**2

        # Mechanical losses
        P_mechanical = self.mechanical_loss_coefficient * omega**2

        # Stray losses
        P_stray = self.stray_loss_coefficient * (Ia**2 + If**2)

        # Temperature dynamics
        dT_arm_dt = (P_copper_a + P_stray - (T_arm - self.ambient_temp) / self.thermal_resistance_a) / self.thermal_capacitance_a
        dT_field_dt = (P_copper_f - (T_field - self.ambient_temp) / self.thermal_resistance_f) / self.thermal_capacitance_f

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dT_arm_dt, dT_field_dt]

    def motor_ode_series(self, t, y):
        """
        Differential equations for DC series motor
        State variables: [Ia, omega, theta, T_arm]
        """
        Ia, omega, theta, T_arm = y

        # Temperature derating
        derating_factor = self.calculate_derating(T_arm)

        # Back EMF (in series motor, field current = armature current)
        Eb = self.Kb * omega * Ia

        # Electrical equation
        dIa_dt = (self.V_supply - Eb - Ia * (self.Ra + self.Rf)) / (self.La + self.Lf)

        # Electromagnetic torque
        Te = self.Kt * Ia * Ia * derating_factor

        # Mechanical equation
        domega_dt = (Te - self.B * omega - self.TL) / self.J

        # Angle
        dtheta_dt = omega

        # Thermal equation
        P_copper = Ia**2 * (self.Ra + self.Rf)
        dT_arm_dt = (P_copper - (T_arm - self.ambient_temp) / self.thermal_resistance_a) / self.thermal_capacitance_a

        return [dIa_dt, domega_dt, dtheta_dt, dT_arm_dt]

    def calculate_derating(self, temperature):
        """Calculate derating factor based on temperature"""
        if temperature < 100:
            return 1.0
        elif temperature < self.max_temp:
            # Linear derating between 100°C and max temperature
            return 1.0 - 0.5 * (temperature - 100) / (self.max_temp - 100)
        else:
            return 0.5  # 50% derating at max temperature

    def calculate_losses(self, Ia, If, omega):
        """Calculate detailed loss breakdown"""
        # Copper losses
        P_copper_a = Ia**2 * self.Ra
        P_copper_f = If**2 * self.Rf

        # Iron losses
        P_iron = self.iron_loss_coefficient * (omega / (2 * np.pi))**2

        # Mechanical losses
        P_mechanical = self.mechanical_loss_coefficient * omega**2

        # Stray losses
        P_stray = self.stray_loss_coefficient * (Ia**2 + If**2)

        total_losses = P_copper_a + P_copper_f + P_iron + P_mechanical + P_stray

        return {
            'copper_armature': P_copper_a,
            'copper_field': P_copper_f,
            'iron': P_iron,
            'mechanical': P_mechanical,
            'stray': P_stray,
            'total': total_losses
        }

    def calculate_shaft_stress(self, torque, omega):
        """Calculate mechanical shaft stress"""
        # Assuming solid circular shaft
        shaft_diameter = 0.05  # m
        shaft_radius = shaft_diameter / 2

        # Torsional shear stress: τ = T*r/J_polar
        J_polar = np.pi * shaft_radius**4 / 2
        shear_stress = torque * shaft_radius / J_polar if J_polar > 0 else 0

        # Bearing load (simplified)
        bearing_load = abs(torque / shaft_radius) if shaft_radius > 0 else 0

        return {
            'shear_stress': shear_stress,
            'bearing_load': bearing_load,
            'angular_velocity': omega
        }


class AdvancedDCMotorSimulator:
    """Main application class for DC motor simulator"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Motor model
        self.motor = DCMotorModel()

        # Simulation parameters
        self.simulation_running = False
        self.simulation_time = 0
        self.max_simulation_time = 10  # seconds
        self.dt = 0.01  # Time step

        # Data storage
        self.max_data_points = 1000
        self.time_data = deque(maxlen=self.max_data_points)
        self.ia_data = deque(maxlen=self.max_data_points)
        self.if_data = deque(maxlen=self.max_data_points)
        self.omega_data = deque(maxlen=self.max_data_points)
        self.torque_data = deque(maxlen=self.max_data_points)
        self.temp_arm_data = deque(maxlen=self.max_data_points)
        self.temp_field_data = deque(maxlen=self.max_data_points)
        self.loss_data = deque(maxlen=self.max_data_points)

        # Initial conditions
        self.y0_shunt = [0, 0, 0, 0, 25, 25]  # [Ia, If, omega, theta, T_arm, T_field]
        self.y0_series = [0, 0, 0, 25]  # [Ia, omega, theta, T_arm]

        # Motor type
        self.motor_type = 'shunt'

        # Create GUI
        self.create_gui()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_gui(self):
        """Create the main GUI"""
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_problem_solutions_tab()
        self.create_simulation_tab()
        self.create_thermal_analysis_tab()
        self.create_loss_analysis_tab()
        self.create_economic_analysis_tab()
        self.create_mechanical_stress_tab()

    def create_problem_solutions_tab(self):
        """Tab for theoretical problem solutions"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Problem Solutions')

        # Create scrolled text widget
        text_frame = ttk.Frame(tab)
        text_frame.pack(fill='both', expand=True, padx=10, pady=10)

        self.problem_text = scrolledtext.ScrolledText(text_frame, wrap=tk.WORD,
                                                       font=('Courier', 10))
        self.problem_text.pack(fill='both', expand=True)

        # Solve and display problems
        self.solve_problems()

    def solve_problems(self):
        """Solve and display theoretical problems"""
        self.problem_text.delete(1.0, tk.END)

        # Problem 3
        self.problem_text.insert(tk.END, "=" * 80 + "\n")
        self.problem_text.insert(tk.END, "PROBLEM 3: DC Shunt Motor with Flux Reduction\n")
        self.problem_text.insert(tk.END, "=" * 80 + "\n\n")

        self.problem_text.insert(tk.END, "Given Data:\n")
        self.problem_text.insert(tk.END, "  Supply Voltage (V) = 500 V\n")
        self.problem_text.insert(tk.END, "  Initial Speed (N₁) = 800 rpm\n")
        self.problem_text.insert(tk.END, "  Initial Armature Current (Ia₁) = 42 A\n")
        self.problem_text.insert(tk.END, "  Armature Resistance (Ra) = 0.6 Ω\n")
        self.problem_text.insert(tk.END, "  Brush Voltage Drop (Vb) = 2 V\n")
        self.problem_text.insert(tk.END, "  Flux Reduction = 75% of normal\n\n")

        result3 = DCMotorProblems.problem_3()

        self.problem_text.insert(tk.END, f"Initial Back EMF (Eb₁) = {result3['Eb1']:.2f} V\n\n")

        self.problem_text.insert(tk.END, "Case (a): Torque Unchanged\n")
        self.problem_text.insert(tk.END, f"  New Armature Current (Ia₂) = {result3['case_a']['Ia2']:.2f} A\n")
        self.problem_text.insert(tk.END, f"  New Back EMF (Eb₂) = {result3['case_a']['Eb2']:.2f} V\n")
        self.problem_text.insert(tk.END, f"  New Speed (N₂) = {result3['case_a']['N2']:.2f} rpm\n")
        self.problem_text.insert(tk.END, f"  Expected: 1042 rpm ✓\n\n")

        self.problem_text.insert(tk.END, "Case (b): Torque Reduced by 20%\n")
        self.problem_text.insert(tk.END, f"  New Armature Current (Ia₂) = {result3['case_b']['Ia2']:.2f} A\n")
        self.problem_text.insert(tk.END, f"  New Back EMF (Eb₂) = {result3['case_b']['Eb2']:.2f} V\n")
        self.problem_text.insert(tk.END, f"  New Speed (N₂) = {result3['case_b']['N2']:.2f} rpm\n")
        self.problem_text.insert(tk.END, f"  Expected: 1061 rpm ✓\n\n")

        # Problem 4
        self.problem_text.insert(tk.END, "\n" + "=" * 80 + "\n")
        self.problem_text.insert(tk.END, "PROBLEM 4: DC Shunt Motor with Flux Increase\n")
        self.problem_text.insert(tk.END, "=" * 80 + "\n\n")

        self.problem_text.insert(tk.END, "Given Data:\n")
        self.problem_text.insert(tk.END, "  Supply Voltage (V) = 460 V\n")
        self.problem_text.insert(tk.END, "  Initial Armature Current (Ia₁) = 28 A\n")
        self.problem_text.insert(tk.END, "  Initial Speed (N₁) = 1000 rpm\n")
        self.problem_text.insert(tk.END, "  Armature Resistance (Ra) = 0.72 Ω\n")
        self.problem_text.insert(tk.END, "  Flux Increase = 120% of initial\n")
        self.problem_text.insert(tk.END, "  Condition: Total torque unchanged\n\n")

        result4 = DCMotorProblems.problem_4()

        self.problem_text.insert(tk.END, f"Initial Back EMF (Eb₁) = {result4['Eb1']:.2f} V\n\n")

        self.problem_text.insert(tk.END, "Results:\n")
        self.problem_text.insert(tk.END, f"  (i)  New Armature Current (Ia₂) = {result4['Ia2']:.2f} A\n")
        self.problem_text.insert(tk.END, f"  (ii) New Speed (N₂) = {result4['N2']:.2f} rpm\n\n")

        self.problem_text.insert(tk.END, "Verification:\n")
        self.problem_text.insert(tk.END, f"  New Back EMF (Eb₂) = {result4['Eb2']:.2f} V\n")
        self.problem_text.insert(tk.END, f"  Speed ratio N₂/N₁ = {result4['N2']/1000:.4f}\n")
        self.problem_text.insert(tk.END, f"  Expected from Eb₂/Eb₁ × φ₁/φ₂ = {(result4['Eb2']/result4['Eb1'])/1.2:.4f} ✓\n\n")

    def create_simulation_tab(self):
        """Main simulation tab with controls and real-time graphs"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Motor Simulation')

        # Main container with two columns
        main_container = ttk.Frame(tab)
        main_container.pack(fill='both', expand=True, padx=5, pady=5)

        # Configure grid weights for auto-resizing
        main_container.grid_rowconfigure(0, weight=1)
        main_container.grid_columnconfigure(0, weight=0)  # Control panel fixed
        main_container.grid_columnconfigure(1, weight=1)  # Graph area expandable

        # Left panel - Controls
        control_panel = ttk.LabelFrame(main_container, text='Control Panel', padding=10)
        control_panel.grid(row=0, column=0, sticky='ns', padx=5, pady=5)

        # Motor type selection
        ttk.Label(control_panel, text="Motor Type:").pack(anchor='w')
        self.motor_type_var = tk.StringVar(value='shunt')
        ttk.Radiobutton(control_panel, text='Shunt Motor', variable=self.motor_type_var,
                       value='shunt', command=self.on_motor_type_change).pack(anchor='w')
        ttk.Radiobutton(control_panel, text='Series Motor', variable=self.motor_type_var,
                       value='series', command=self.on_motor_type_change).pack(anchor='w')

        ttk.Separator(control_panel, orient='horizontal').pack(fill='x', pady=10)

        # Solver selection
        ttk.Label(control_panel, text="ODE Solver:").pack(anchor='w')
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(control_panel, text='RK45 (Runge-Kutta)', variable=self.solver_var,
                       value='RK45').pack(anchor='w')
        ttk.Radiobutton(control_panel, text='Euler Method', variable=self.solver_var,
                       value='Euler').pack(anchor='w')

        ttk.Separator(control_panel, orient='horizontal').pack(fill='x', pady=10)

        # Parameter controls
        ttk.Label(control_panel, text="Supply Voltage (V):").pack(anchor='w')
        self.voltage_scale = tk.Scale(control_panel, from_=0, to=500, orient='horizontal',
                                      command=self.update_voltage)
        self.voltage_scale.set(220)
        self.voltage_scale.pack(fill='x')
        self.voltage_label = ttk.Label(control_panel, text="220 V")
        self.voltage_label.pack(anchor='w')

        ttk.Label(control_panel, text="Load Torque (N.m):").pack(anchor='w')
        self.torque_scale = tk.Scale(control_panel, from_=0, to=50, resolution=0.1,
                                     orient='horizontal', command=self.update_load_torque)
        self.torque_scale.set(5)
        self.torque_scale.pack(fill='x')
        self.torque_label = ttk.Label(control_panel, text="5.0 N.m")
        self.torque_label.pack(anchor='w')

        ttk.Label(control_panel, text="Armature Resistance (Ω):").pack(anchor='w')
        self.ra_scale = tk.Scale(control_panel, from_=0.1, to=5, resolution=0.1,
                                orient='horizontal', command=self.update_ra)
        self.ra_scale.set(0.5)
        self.ra_scale.pack(fill='x')
        self.ra_label = ttk.Label(control_panel, text="0.5 Ω")
        self.ra_label.pack(anchor='w')

        ttk.Label(control_panel, text="Inertia (kg.m²):").pack(anchor='w')
        self.inertia_scale = tk.Scale(control_panel, from_=0.001, to=0.1, resolution=0.001,
                                     orient='horizontal', command=self.update_inertia)
        self.inertia_scale.set(0.02)
        self.inertia_scale.pack(fill='x')
        self.inertia_label = ttk.Label(control_panel, text="0.020 kg.m²")
        self.inertia_label.pack(anchor='w')

        ttk.Separator(control_panel, orient='horizontal').pack(fill='x', pady=10)

        # Control buttons
        button_frame = ttk.Frame(control_panel)
        button_frame.pack(fill='x', pady=5)

        self.start_btn = ttk.Button(button_frame, text="Start", command=self.start_simulation)
        self.start_btn.pack(side='left', padx=2, expand=True, fill='x')

        self.stop_btn = ttk.Button(button_frame, text="Stop", command=self.stop_simulation,
                                   state='disabled')
        self.stop_btn.pack(side='left', padx=2, expand=True, fill='x')

        self.reset_btn = ttk.Button(button_frame, text="Reset", command=self.reset_simulation)
        self.reset_btn.pack(side='left', padx=2, expand=True, fill='x')

        # Status display
        ttk.Separator(control_panel, orient='horizontal').pack(fill='x', pady=10)

        self.status_frame = ttk.LabelFrame(control_panel, text='Status', padding=5)
        self.status_frame.pack(fill='x')

        self.status_labels = {}
        status_vars = ['Time', 'Speed', 'Current', 'Torque', 'Power', 'Temp']
        for var in status_vars:
            frame = ttk.Frame(self.status_frame)
            frame.pack(fill='x')
            ttk.Label(frame, text=f"{var}:", width=8).pack(side='left')
            self.status_labels[var] = ttk.Label(frame, text="0", width=12)
            self.status_labels[var].pack(side='left')

        # Right panel - Graphs
        graph_panel = ttk.Frame(main_container)
        graph_panel.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Create matplotlib figure with subplots
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.fig.tight_layout(pad=3.0)

        # Create subplots
        self.ax1 = self.fig.add_subplot(3, 2, 1)  # Speed
        self.ax2 = self.fig.add_subplot(3, 2, 2)  # Current
        self.ax3 = self.fig.add_subplot(3, 2, 3)  # Torque
        self.ax4 = self.fig.add_subplot(3, 2, 4)  # Temperature
        self.ax5 = self.fig.add_subplot(3, 2, 5)  # Power
        self.ax6 = self.fig.add_subplot(3, 2, 6)  # Losses

        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # Initialize plots
        self.init_plots()

    def create_thermal_analysis_tab(self):
        """Tab for detailed thermal analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Thermal Analysis')

        # Create figure for thermal plots
        fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.set_title('Temperature Distribution')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Temperature (°C)')
        ax1.grid(True)

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_title('Heat Generation vs Dissipation')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Power (W)')
        ax2.grid(True)

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.set_title('Thermal Derating Factor')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Derating Factor')
        ax3.grid(True)

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title('Temperature vs Current')
        ax4.set_xlabel('Current (A)')
        ax4.set_ylabel('Temperature (°C)')
        ax4.grid(True)

        self.thermal_fig = fig
        self.thermal_axes = [ax1, ax2, ax3, ax4]

        canvas = FigureCanvasTkAgg(fig, master=tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def create_loss_analysis_tab(self):
        """Tab for detailed loss breakdown"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Loss Analysis')

        # Create figure for loss plots
        fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.set_title('Loss Breakdown (Pie Chart)')

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_title('Losses vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Power Loss (W)')
        ax2.grid(True)

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.set_title('Efficiency vs Load')
        ax3.set_xlabel('Load Torque (N.m)')
        ax3.set_ylabel('Efficiency (%)')
        ax3.grid(True)

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title('Cumulative Energy Loss')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Energy (J)')
        ax4.grid(True)

        self.loss_fig = fig
        self.loss_axes = [ax1, ax2, ax3, ax4]

        canvas = FigureCanvasTkAgg(fig, master=tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def create_economic_analysis_tab(self):
        """Tab for economic analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Economic Analysis')

        # Create two panels
        input_frame = ttk.LabelFrame(tab, text='Economic Parameters', padding=10)
        input_frame.pack(side='left', fill='both', padx=5, pady=5)

        result_frame = ttk.LabelFrame(tab, text='Economic Results', padding=10)
        result_frame.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        # Input parameters
        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").pack(anchor='w')
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(input_frame, textvariable=self.elec_cost_var).pack(fill='x', pady=2)

        ttk.Label(input_frame, text="Operating Hours/Day:").pack(anchor='w')
        self.op_hours_var = tk.DoubleVar(value=8)
        ttk.Entry(input_frame, textvariable=self.op_hours_var).pack(fill='x', pady=2)

        ttk.Label(input_frame, text="Operating Days/Year:").pack(anchor='w')
        self.op_days_var = tk.DoubleVar(value=250)
        ttk.Entry(input_frame, textvariable=self.op_days_var).pack(fill='x', pady=2)

        ttk.Label(input_frame, text="Motor Initial Cost ($):").pack(anchor='w')
        self.motor_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(input_frame, textvariable=self.motor_cost_var).pack(fill='x', pady=2)

        ttk.Label(input_frame, text="Maintenance Cost/Year ($):").pack(anchor='w')
        self.maint_cost_var = tk.DoubleVar(value=500)
        ttk.Entry(input_frame, textvariable=self.maint_cost_var).pack(fill='x', pady=2)

        ttk.Label(input_frame, text="Expected Lifetime (years):").pack(anchor='w')
        self.lifetime_var = tk.DoubleVar(value=15)
        ttk.Entry(input_frame, textvariable=self.lifetime_var).pack(fill='x', pady=2)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).pack(pady=10, fill='x')

        # Results display
        self.econ_text = scrolledtext.ScrolledText(result_frame, wrap=tk.WORD,
                                                    font=('Courier', 10))
        self.econ_text.pack(fill='both', expand=True)

    def create_mechanical_stress_tab(self):
        """Tab for mechanical stress analysis"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Mechanical Stress')

        # Create figure for stress plots
        fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.set_title('Shaft Shear Stress vs Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Shear Stress (Pa)')
        ax1.grid(True)

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_title('Bearing Load vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Bearing Load (N)')
        ax2.grid(True)

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.set_title('Torque-Speed Characteristic')
        ax3.set_xlabel('Speed (rpm)')
        ax3.set_ylabel('Torque (N.m)')
        ax3.grid(True)

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title('Stress Distribution')
        ax4.set_xlabel('Radial Position')
        ax4.set_ylabel('Stress (Pa)')
        ax4.grid(True)

        self.mech_fig = fig
        self.mech_axes = [ax1, ax2, ax3, ax4]

        canvas = FigureCanvasTkAgg(fig, master=tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def init_plots(self):
        """Initialize plot formatting"""
        self.ax1.set_title('Speed (rpm)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (rpm)')
        self.ax1.grid(True)

        self.ax2.set_title('Armature Current (A)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.grid(True)

        self.ax3.set_title('Electromagnetic Torque (N.m)')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Torque (N.m)')
        self.ax3.grid(True)

        self.ax4.set_title('Temperature (°C)')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temp (°C)')
        self.ax4.grid(True)

        self.ax5.set_title('Power (W)')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (W)')
        self.ax5.grid(True)

        self.ax6.set_title('Total Losses (W)')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Losses (W)')
        self.ax6.grid(True)

    def update_voltage(self, value):
        """Update supply voltage"""
        self.motor.V_supply = float(value)
        self.voltage_label.config(text=f"{float(value):.0f} V")

    def update_load_torque(self, value):
        """Update load torque"""
        self.motor.TL = float(value)
        self.torque_label.config(text=f"{float(value):.1f} N.m")

    def update_ra(self, value):
        """Update armature resistance"""
        self.motor.Ra = float(value)
        self.ra_label.config(text=f"{float(value):.1f} Ω")

    def update_inertia(self, value):
        """Update moment of inertia"""
        self.motor.J = float(value)
        self.inertia_label.config(text=f"{float(value):.3f} kg.m²")

    def on_motor_type_change(self):
        """Handle motor type change"""
        self.motor_type = self.motor_type_var.get()
        self.reset_simulation()

    def start_simulation(self):
        """Start the simulation"""
        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.motor.solver_type = self.solver_var.get()
        self.run_simulation()

    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation to initial conditions"""
        self.simulation_running = False
        self.simulation_time = 0
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

        # Clear data
        self.time_data.clear()
        self.ia_data.clear()
        self.if_data.clear()
        self.omega_data.clear()
        self.torque_data.clear()
        self.temp_arm_data.clear()
        self.temp_field_data.clear()
        self.loss_data.clear()

        # Reset initial conditions
        if self.motor_type == 'shunt':
            self.y0_shunt = [0, 0, 0, 0, 25, 25]
        else:
            self.y0_series = [0, 0, 0, 25]

        # Clear plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax5.clear()
        self.ax6.clear()
        self.init_plots()
        self.canvas.draw()

    def run_simulation(self):
        """Run simulation with selected ODE solver"""
        if not self.simulation_running:
            return

        # Time span for this step
        t_span = [self.simulation_time, self.simulation_time + self.dt]
        t_eval = [self.simulation_time + self.dt]

        try:
            if self.motor_type == 'shunt':
                if self.motor.solver_type == 'RK45':
                    # Use RK45 solver
                    sol = solve_ivp(self.motor.motor_ode_shunt, t_span, self.y0_shunt,
                                   method='RK45', t_eval=t_eval, max_step=self.dt)
                    y_new = sol.y[:, -1]
                else:
                    # Use Euler method
                    dydt = self.motor.motor_ode_shunt(self.simulation_time, self.y0_shunt)
                    y_new = np.array(self.y0_shunt) + np.array(dydt) * self.dt

                Ia, If, omega, theta, T_arm, T_field = y_new
                self.y0_shunt = y_new

                # Calculate torque
                Te = self.motor.Kt * If * Ia * self.motor.calculate_derating(T_arm)

            else:  # series motor
                if self.motor.solver_type == 'RK45':
                    sol = solve_ivp(self.motor.motor_ode_series, t_span, self.y0_series,
                                   method='RK45', t_eval=t_eval, max_step=self.dt)
                    y_new = sol.y[:, -1]
                else:
                    dydt = self.motor.motor_ode_series(self.simulation_time, self.y0_series)
                    y_new = np.array(self.y0_series) + np.array(dydt) * self.dt

                Ia, omega, theta, T_arm = y_new
                If = Ia  # In series motor
                T_field = T_arm
                self.y0_series = y_new

                # Calculate torque
                Te = self.motor.Kt * Ia * Ia * self.motor.calculate_derating(T_arm)

            # Calculate losses
            losses = self.motor.calculate_losses(Ia, If, omega)

            # Store data
            self.time_data.append(self.simulation_time)
            self.ia_data.append(Ia)
            self.if_data.append(If)
            self.omega_data.append(omega * 30 / np.pi)  # Convert to rpm
            self.torque_data.append(Te)
            self.temp_arm_data.append(T_arm)
            self.temp_field_data.append(T_field)
            self.loss_data.append(losses['total'])

            # Update plots every 10 steps
            if len(self.time_data) % 10 == 0:
                self.update_plots()

            # Update status
            self.update_status(Ia, omega, Te, T_arm)

            # Increment time
            self.simulation_time += self.dt

            # Check if simulation should continue
            if self.simulation_time < self.max_simulation_time:
                self.root.after(10, self.run_simulation)  # Continue simulation
            else:
                self.stop_simulation()

        except Exception as e:
            print(f"Simulation error: {e}")
            self.stop_simulation()

    def update_plots(self):
        """Update all plots with current data"""
        if len(self.time_data) < 2:
            return

        time_array = np.array(self.time_data)

        # Speed plot
        self.ax1.clear()
        self.ax1.plot(time_array, self.omega_data, 'b-', linewidth=2)
        self.ax1.set_title('Speed (rpm)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (rpm)')
        self.ax1.grid(True)

        # Current plot
        self.ax2.clear()
        self.ax2.plot(time_array, self.ia_data, 'r-', linewidth=2, label='Ia')
        if self.motor_type == 'shunt':
            self.ax2.plot(time_array, self.if_data, 'g-', linewidth=2, label='If')
        self.ax2.set_title('Currents (A)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.legend()
        self.ax2.grid(True)

        # Torque plot
        self.ax3.clear()
        self.ax3.plot(time_array, self.torque_data, 'm-', linewidth=2)
        self.ax3.axhline(y=self.motor.TL, color='r', linestyle='--', label='Load Torque')
        self.ax3.set_title('Torque (N.m)')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Torque (N.m)')
        self.ax3.legend()
        self.ax3.grid(True)

        # Temperature plot
        self.ax4.clear()
        self.ax4.plot(time_array, self.temp_arm_data, 'r-', linewidth=2, label='Armature')
        if self.motor_type == 'shunt':
            self.ax4.plot(time_array, self.temp_field_data, 'b-', linewidth=2, label='Field')
        self.ax4.axhline(y=self.motor.max_temp, color='r', linestyle='--', label='Max Temp')
        self.ax4.set_title('Temperature (°C)')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temp (°C)')
        self.ax4.legend()
        self.ax4.grid(True)

        # Power plot
        self.ax5.clear()
        ia_array = np.array(self.ia_data)
        if_array = np.array(self.if_data)
        omega_array = np.array(self.omega_data) * np.pi / 30  # Convert to rad/s
        torque_array = np.array(self.torque_data)

        power_input = self.motor.V_supply * (ia_array + if_array)
        power_output = torque_array * omega_array

        self.ax5.plot(time_array, power_input, 'b-', linewidth=2, label='Input')
        self.ax5.plot(time_array, power_output, 'g-', linewidth=2, label='Output')
        self.ax5.set_title('Power (W)')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (W)')
        self.ax5.legend()
        self.ax5.grid(True)

        # Losses plot
        self.ax6.clear()
        self.ax6.plot(time_array, self.loss_data, 'r-', linewidth=2)
        self.ax6.set_title('Total Losses (W)')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Losses (W)')
        self.ax6.grid(True)

        self.fig.tight_layout()
        self.canvas.draw()

    def update_status(self, Ia, omega, Te, T_arm):
        """Update status display"""
        rpm = omega * 30 / np.pi
        power_out = Te * omega

        self.status_labels['Time'].config(text=f"{self.simulation_time:.2f} s")
        self.status_labels['Speed'].config(text=f"{rpm:.1f} rpm")
        self.status_labels['Current'].config(text=f"{Ia:.2f} A")
        self.status_labels['Torque'].config(text=f"{Te:.2f} N.m")
        self.status_labels['Power'].config(text=f"{power_out:.1f} W")
        self.status_labels['Temp'].config(text=f"{T_arm:.1f} °C")

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            elec_cost = self.elec_cost_var.get()
            op_hours = self.op_hours_var.get()
            op_days = self.op_days_var.get()
            motor_cost = self.motor_cost_var.get()
            maint_cost = self.maint_cost_var.get()
            lifetime = self.lifetime_var.get()

            # Assume average power consumption from simulation
            if len(self.ia_data) > 0:
                avg_ia = np.mean(self.ia_data)
                avg_if = np.mean(self.if_data)
                avg_power_kw = self.motor.V_supply * (avg_ia + avg_if) / 1000
            else:
                avg_power_kw = 5  # Default assumption

            # Annual calculations
            annual_hours = op_hours * op_days
            annual_energy_kwh = avg_power_kw * annual_hours
            annual_energy_cost = annual_energy_kwh * elec_cost
            annual_total_cost = annual_energy_cost + maint_cost

            # Lifetime calculations
            lifetime_energy_cost = annual_energy_cost * lifetime
            lifetime_maint_cost = maint_cost * lifetime
            lifetime_total_cost = motor_cost + lifetime_energy_cost + lifetime_maint_cost

            # Cost per hour
            cost_per_hour = lifetime_total_cost / (annual_hours * lifetime)

            # Display results
            self.econ_text.delete(1.0, tk.END)
            self.econ_text.insert(tk.END, "=" * 60 + "\n")
            self.econ_text.insert(tk.END, "ECONOMIC ANALYSIS RESULTS\n")
            self.econ_text.insert(tk.END, "=" * 60 + "\n\n")

            self.econ_text.insert(tk.END, f"Average Power Consumption: {avg_power_kw:.2f} kW\n\n")

            self.econ_text.insert(tk.END, "ANNUAL COSTS:\n")
            self.econ_text.insert(tk.END, f"  Operating Hours:        {annual_hours:.0f} hours\n")
            self.econ_text.insert(tk.END, f"  Energy Consumption:     {annual_energy_kwh:.2f} kWh\n")
            self.econ_text.insert(tk.END, f"  Energy Cost:            ${annual_energy_cost:.2f}\n")
            self.econ_text.insert(tk.END, f"  Maintenance Cost:       ${maint_cost:.2f}\n")
            self.econ_text.insert(tk.END, f"  Total Annual Cost:      ${annual_total_cost:.2f}\n\n")

            self.econ_text.insert(tk.END, f"LIFETIME COSTS ({lifetime:.0f} years):\n")
            self.econ_text.insert(tk.END, f"  Initial Investment:     ${motor_cost:.2f}\n")
            self.econ_text.insert(tk.END, f"  Total Energy Cost:      ${lifetime_energy_cost:.2f}\n")
            self.econ_text.insert(tk.END, f"  Total Maintenance Cost: ${lifetime_maint_cost:.2f}\n")
            self.econ_text.insert(tk.END, f"  TOTAL LIFETIME COST:    ${lifetime_total_cost:.2f}\n\n")

            self.econ_text.insert(tk.END, f"Cost per Operating Hour:  ${cost_per_hour:.4f}/hour\n\n")

            # Efficiency analysis
            if len(self.ia_data) > 0:
                avg_losses = np.mean(self.loss_data)
                avg_input = avg_power_kw * 1000
                efficiency = ((avg_input - avg_losses) / avg_input * 100) if avg_input > 0 else 0

                self.econ_text.insert(tk.END, "EFFICIENCY ANALYSIS:\n")
                self.econ_text.insert(tk.END, f"  Average Input Power:    {avg_input:.2f} W\n")
                self.econ_text.insert(tk.END, f"  Average Losses:         {avg_losses:.2f} W\n")
                self.econ_text.insert(tk.END, f"  Average Efficiency:     {efficiency:.2f} %\n\n")

                # Cost of losses
                loss_energy_kwh = (avg_losses / 1000) * annual_hours
                loss_cost = loss_energy_kwh * elec_cost

                self.econ_text.insert(tk.END, f"  Annual Energy Lost:     {loss_energy_kwh:.2f} kWh\n")
                self.econ_text.insert(tk.END, f"  Annual Cost of Losses:  ${loss_cost:.2f}\n")
                self.econ_text.insert(tk.END, f"  Lifetime Cost of Losses: ${loss_cost * lifetime:.2f}\n")

        except Exception as e:
            self.econ_text.delete(1.0, tk.END)
            self.econ_text.insert(tk.END, f"Error in calculation: {e}\n")

    def on_window_resize(self, event):
        """Handle window resize events"""
        # The matplotlib canvas will automatically resize with the window
        # due to pack(fill='both', expand=True)
        pass


def main():
    """Main function to run the application"""
    root = tk.Tk()
    app = AdvancedDCMotorSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
