"""
Advanced Transformer Parallel Load Distribution Simulator
with Multi-Physics Simulation, Dynamic Analysis, and Economic Assessment

Features:
- Parallel transformer load distribution calculation
- Multi-physics simulation (electromagnetic, thermal, mechanical)
- Real-time ODE solvers (RK45, Euler)
- Dynamic visualization
- Economic analysis
- Thermal derating and advanced controls
- Auto-scaling GUI
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

class TransformerLoadDistribution:
    """Core calculation engine for transformer load distribution"""

    def __init__(self):
        # Transformer A parameters
        self.S_A_rated = 2000  # kVA
        self.R_A_percent = 2.0  # %
        self.X_A_percent = 8.0  # %

        # Transformer B parameters
        self.S_B_rated = 4000  # kVA
        self.R_B_percent = 1.6  # %
        self.X_B_percent = 3.0  # %

        # Load parameters
        self.S_load = 5000  # kVA
        self.pf = 0.8  # power factor
        self.voltage = 11.0  # kV (assumed)

        # Physical parameters
        self.ambient_temp = 25  # °C
        self.cooling_coefficient = 0.05
        self.thermal_time_constant = 1800  # seconds

        # Results storage
        self.S_A = 0
        self.S_B = 0
        self.I_A = 0
        self.I_B = 0
        self.temp_A = self.ambient_temp
        self.temp_B = self.ambient_temp

    def calculate_impedances(self):
        """Calculate transformer impedances on common base"""
        # Common base = total load
        base_S = self.S_load

        # Convert to common base
        Z_A = complex(self.R_A_percent/100, self.X_A_percent/100) * (base_S / self.S_A_rated)
        Z_B = complex(self.R_B_percent/100, self.X_B_percent/100) * (base_S / self.S_B_rated)

        return Z_A, Z_B

    def calculate_load_distribution(self):
        """Calculate load distribution between parallel transformers"""
        Z_A, Z_B = self.calculate_impedances()

        # Calculate magnitudes
        Z_A_mag = abs(Z_A)
        Z_B_mag = abs(Z_B)

        # Load distribution (inversely proportional to impedance)
        # Current distribution: I_A/I_B = Z_B/Z_A
        # For equal voltages: S_A/S_B = I_A/I_B (approximately for similar power factors)

        # More accurate method using complex power
        # S_A = S_total * Z_B / (Z_A + Z_B)
        # S_B = S_total * Z_A / (Z_A + Z_B)

        total_admittance = 1/Z_A + 1/Z_B
        Y_A = 1/Z_A
        Y_B = 1/Z_B

        # Load current (total)
        I_load = self.S_load * 1000 / (math.sqrt(3) * self.voltage * 1000)  # Amps

        # Current distribution (admittance ratio)
        I_A_mag = abs(Y_A / total_admittance) * I_load
        I_B_mag = abs(Y_B / total_admittance) * I_load

        # Power distribution
        self.S_A = math.sqrt(3) * self.voltage * 1000 * I_A_mag / 1000  # kVA
        self.S_B = math.sqrt(3) * self.voltage * 1000 * I_B_mag / 1000  # kVA

        self.I_A = I_A_mag
        self.I_B = I_B_mag

        # Calculate loading percentages
        loading_A = (self.S_A / self.S_A_rated) * 100
        loading_B = (self.S_B / self.S_B_rated) * 100

        return {
            'S_A': self.S_A,
            'S_B': self.S_B,
            'I_A': self.I_A,
            'I_B': self.I_B,
            'loading_A': loading_A,
            'loading_B': loading_B,
            'Z_A': Z_A,
            'Z_B': Z_B
        }

    def calculate_losses(self, S_actual, S_rated, R_percent):
        """Calculate transformer losses"""
        # Copper losses (proportional to square of loading)
        loading_ratio = S_actual / S_rated
        copper_loss = (loading_ratio ** 2) * (R_percent / 100) * S_rated

        # Iron losses (approximately constant)
        iron_loss = 0.002 * S_rated  # Assumed 0.2% of rated power

        # Stray load losses
        stray_loss = 0.001 * S_rated * (loading_ratio ** 1.8)

        # Mechanical losses (friction, cooling)
        mechanical_loss = 0.001 * S_rated

        total_loss = copper_loss + iron_loss + stray_loss + mechanical_loss

        return {
            'copper': copper_loss,
            'iron': iron_loss,
            'stray': stray_loss,
            'mechanical': mechanical_loss,
            'total': total_loss
        }

    def calculate_temperature(self, S_actual, S_rated, time_step=1.0):
        """Calculate transformer temperature using thermal model"""
        loading_ratio = S_actual / S_rated

        # Heat generation (proportional to losses)
        heat_generation = loading_ratio ** 2

        # Simplified thermal differential equation:
        # dT/dt = (1/τ) * (ΔT_rated * loading² - (T - T_ambient))
        # where τ is thermal time constant

        delta_T_rated = 65  # °C temperature rise at rated load

        return delta_T_rated * (loading_ratio ** 2) + self.ambient_temp

    def calculate_efficiency(self, S_actual, losses):
        """Calculate transformer efficiency"""
        P_out = S_actual * self.pf  # kW
        P_in = P_out + losses['total']

        if P_in > 0:
            efficiency = (P_out / P_in) * 100
        else:
            efficiency = 0

        return efficiency


class MultiPhysicsSimulator:
    """Multi-physics simulation engine"""

    def __init__(self, transformer_calc):
        self.transformer = transformer_calc
        self.time_history = []
        self.temperature_A_history = []
        self.temperature_B_history = []
        self.torque_history = []
        self.losses_history = []

    def thermal_model_ode(self, t, y, loading_ratio, S_rated):
        """
        Thermal ODE model
        y[0] = Temperature
        dy/dt = (1/τ) * (ΔT_rated * loading² + T_ambient - T)
        """
        T = y[0]
        tau = self.transformer.thermal_time_constant
        delta_T_rated = 65  # °C
        T_ambient = self.transformer.ambient_temp

        dT_dt = (1/tau) * (delta_T_rated * (loading_ratio ** 2) + T_ambient - T)

        return [dT_dt]

    def electromagnetic_model(self, V, I, Z):
        """Electromagnetic model - voltage, current, impedance relationships"""
        # Voltage drop across impedance
        V_drop = I * abs(Z)

        # Magnetic flux (simplified)
        flux = V / (2 * math.pi * 50)  # Assuming 50 Hz

        # Magnetic field intensity
        H = I / 0.1  # Simplified, assuming magnetic path length

        return {
            'voltage_drop': V_drop,
            'flux': flux,
            'field_intensity': H
        }

    def mechanical_model(self, S_actual, S_rated):
        """Mechanical stress model"""
        # Torque calculation (simplified)
        loading_ratio = S_actual / S_rated

        # Shaft torque (proportional to power)
        torque = S_actual * 1000 / (2 * math.pi * 50)  # Nm (simplified)

        # Bearing load (approximated)
        bearing_load = torque * 0.1  # N

        # Vibration level
        vibration = loading_ratio * 0.5  # mm/s (simplified)

        return {
            'torque': torque,
            'bearing_load': bearing_load,
            'vibration': vibration
        }

    def simulate_dynamic_response(self, duration=100, method='RK45'):
        """Simulate dynamic thermal response"""
        results = self.transformer.calculate_load_distribution()

        # Initial conditions
        T0_A = self.transformer.ambient_temp
        T0_B = self.transformer.ambient_temp

        # Loading ratios
        loading_A = results['S_A'] / self.transformer.S_A_rated
        loading_B = results['S_B'] / self.transformer.S_B_rated

        # Time span
        t_span = (0, duration)
        t_eval = np.linspace(0, duration, 100)

        # Solve for Transformer A
        if method == 'RK45':
            sol_A = solve_ivp(
                lambda t, y: self.thermal_model_ode(t, y, loading_A, self.transformer.S_A_rated),
                t_span, [T0_A], method='RK45', t_eval=t_eval
            )
            sol_B = solve_ivp(
                lambda t, y: self.thermal_model_ode(t, y, loading_B, self.transformer.S_B_rated),
                t_span, [T0_B], method='RK45', t_eval=t_eval
            )
        else:  # Euler method
            sol_A = self.euler_method(
                lambda t, y: self.thermal_model_ode(t, y, loading_A, self.transformer.S_A_rated),
                t_span, [T0_A], t_eval
            )
            sol_B = self.euler_method(
                lambda t, y: self.thermal_model_ode(t, y, loading_B, self.transformer.S_B_rated),
                t_span, [T0_B], t_eval
            )

        self.time_history = sol_A.t if method == 'RK45' else t_eval
        self.temperature_A_history = sol_A.y[0] if method == 'RK45' else sol_A
        self.temperature_B_history = sol_B.y[0] if method == 'RK45' else sol_B

        return {
            'time': self.time_history,
            'temp_A': self.temperature_A_history,
            'temp_B': self.temperature_B_history
        }

    def euler_method(self, f, t_span, y0, t_eval):
        """Simple Euler method for ODE solving"""
        y = np.zeros((len(y0), len(t_eval)))
        y[:, 0] = y0

        for i in range(1, len(t_eval)):
            dt = t_eval[i] - t_eval[i-1]
            dy = f(t_eval[i-1], y[:, i-1])
            y[:, i] = y[:, i-1] + np.array(dy) * dt

        # Create result object similar to solve_ivp
        class EulerResult:
            def __init__(self, t, y):
                self.t = t
                self.y = y

        return EulerResult(t_eval, y)


class EconomicAnalyzer:
    """Economic analysis module"""

    def __init__(self, transformer_calc):
        self.transformer = transformer_calc
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost_per_year = 5000  # $
        self.transformer_A_cost = 150000  # $
        self.transformer_B_cost = 280000  # $

    def calculate_operating_cost(self, hours_per_year=8760):
        """Calculate annual operating cost"""
        results = self.transformer.calculate_load_distribution()

        # Calculate losses
        losses_A = self.transformer.calculate_losses(
            results['S_A'], self.transformer.S_A_rated, self.transformer.R_A_percent
        )
        losses_B = self.transformer.calculate_losses(
            results['S_B'], self.transformer.S_B_rated, self.transformer.R_B_percent
        )

        # Energy cost
        total_losses_kW = losses_A['total'] + losses_B['total']
        annual_energy_loss_kWh = total_losses_kW * hours_per_year
        annual_energy_cost = annual_energy_loss_kWh * self.electricity_cost

        # Total operating cost
        total_operating_cost = annual_energy_cost + self.maintenance_cost_per_year

        return {
            'energy_loss_kWh': annual_energy_loss_kWh,
            'energy_cost': annual_energy_cost,
            'maintenance_cost': self.maintenance_cost_per_year,
            'total_operating_cost': total_operating_cost,
            'losses_A': losses_A,
            'losses_B': losses_B
        }

    def calculate_roi(self, years=10):
        """Calculate return on investment"""
        initial_investment = self.transformer_A_cost + self.transformer_B_cost

        operating_cost = self.calculate_operating_cost()
        annual_cost = operating_cost['total_operating_cost']

        total_cost_over_years = initial_investment + (annual_cost * years)

        # Revenue (simplified - based on energy delivered)
        energy_delivered_kWh = self.transformer.S_load * self.transformer.pf * 8760
        annual_revenue = energy_delivered_kWh * self.electricity_cost * 1.5  # 1.5x markup
        total_revenue = annual_revenue * years

        roi = ((total_revenue - total_cost_over_years) / initial_investment) * 100
        payback_period = initial_investment / (annual_revenue - annual_cost)

        return {
            'roi': roi,
            'payback_period': payback_period,
            'total_cost': total_cost_over_years,
            'total_revenue': total_revenue
        }


class AdvancedTransformerGUI:
    """Advanced GUI with Tkinter"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Transformer Load Distribution Simulator")
        self.root.geometry("1400x900")

        # Core objects
        self.transformer = TransformerLoadDistribution()
        self.simulator = MultiPhysicsSimulator(self.transformer)
        self.economics = EconomicAnalyzer(self.transformer)

        # Simulation state
        self.is_running = False
        self.simulation_thread = None
        self.ode_method = 'RK45'

        # Setup GUI
        self.setup_menu()
        self.setup_notebook()
        self.setup_control_panel()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def setup_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_command(label="Load Configuration", command=self.load_config)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Run", command=self.start_simulation)
        sim_menu.add_command(label="Stop", command=self.stop_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Documentation", command=self.show_docs)

    def setup_notebook(self):
        """Create tabbed interface"""
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_input = ttk.Frame(self.notebook)
        self.tab_results = ttk.Frame(self.notebook)
        self.tab_dynamics = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_economics = ttk.Frame(self.notebook)
        self.tab_losses = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_input, text="Input Parameters")
        self.notebook.add(self.tab_results, text="Load Distribution")
        self.notebook.add(self.tab_dynamics, text="Dynamic Simulation")
        self.notebook.add(self.tab_thermal, text="Thermal Analysis")
        self.notebook.add(self.tab_economics, text="Economic Analysis")
        self.notebook.add(self.tab_losses, text="Loss Breakdown")

        # Setup each tab
        self.setup_input_tab()
        self.setup_results_tab()
        self.setup_dynamics_tab()
        self.setup_thermal_tab()
        self.setup_economics_tab()
        self.setup_losses_tab()

    def setup_input_tab(self):
        """Setup input parameters tab"""
        # Create scrollable frame
        canvas = tk.Canvas(self.tab_input)
        scrollbar = ttk.Scrollbar(self.tab_input, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Transformer A parameters
        frame_A = ttk.LabelFrame(scrollable_frame, text="Transformer A Parameters", padding=10)
        frame_A.grid(row=0, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(frame_A, text="Rated Power (kVA):").grid(row=0, column=0, sticky='w', pady=5)
        self.entry_S_A = ttk.Entry(frame_A)
        self.entry_S_A.insert(0, "2000")
        self.entry_S_A.grid(row=0, column=1, pady=5)

        ttk.Label(frame_A, text="Resistance (%):").grid(row=1, column=0, sticky='w', pady=5)
        self.slider_R_A = ttk.Scale(frame_A, from_=0, to=10, orient='horizontal', length=200)
        self.slider_R_A.set(2.0)
        self.slider_R_A.grid(row=1, column=1, pady=5)
        self.label_R_A = ttk.Label(frame_A, text="2.0%")
        self.label_R_A.grid(row=1, column=2, pady=5)
        self.slider_R_A.configure(command=lambda v: self.label_R_A.config(text=f"{float(v):.2f}%"))

        ttk.Label(frame_A, text="Reactance (%):").grid(row=2, column=0, sticky='w', pady=5)
        self.slider_X_A = ttk.Scale(frame_A, from_=0, to=20, orient='horizontal', length=200)
        self.slider_X_A.set(8.0)
        self.slider_X_A.grid(row=2, column=1, pady=5)
        self.label_X_A = ttk.Label(frame_A, text="8.0%")
        self.label_X_A.grid(row=2, column=2, pady=5)
        self.slider_X_A.configure(command=lambda v: self.label_X_A.config(text=f"{float(v):.2f}%"))

        # Transformer B parameters
        frame_B = ttk.LabelFrame(scrollable_frame, text="Transformer B Parameters", padding=10)
        frame_B.grid(row=1, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(frame_B, text="Rated Power (kVA):").grid(row=0, column=0, sticky='w', pady=5)
        self.entry_S_B = ttk.Entry(frame_B)
        self.entry_S_B.insert(0, "4000")
        self.entry_S_B.grid(row=0, column=1, pady=5)

        ttk.Label(frame_B, text="Resistance (%):").grid(row=1, column=0, sticky='w', pady=5)
        self.slider_R_B = ttk.Scale(frame_B, from_=0, to=10, orient='horizontal', length=200)
        self.slider_R_B.set(1.6)
        self.slider_R_B.grid(row=1, column=1, pady=5)
        self.label_R_B = ttk.Label(frame_B, text="1.6%")
        self.label_R_B.grid(row=1, column=2, pady=5)
        self.slider_R_B.configure(command=lambda v: self.label_R_B.config(text=f"{float(v):.2f}%"))

        ttk.Label(frame_B, text="Reactance (%):").grid(row=2, column=0, sticky='w', pady=5)
        self.slider_X_B = ttk.Scale(frame_B, from_=0, to=20, orient='horizontal', length=200)
        self.slider_X_B.set(3.0)
        self.slider_X_B.grid(row=2, column=1, pady=5)
        self.label_X_B = ttk.Label(frame_B, text="3.0%")
        self.label_X_B.grid(row=2, column=2, pady=5)
        self.slider_X_B.configure(command=lambda v: self.label_X_B.config(text=f"{float(v):.2f}%"))

        # Load parameters
        frame_load = ttk.LabelFrame(scrollable_frame, text="Load Parameters", padding=10)
        frame_load.grid(row=2, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(frame_load, text="Load Power (kVA):").grid(row=0, column=0, sticky='w', pady=5)
        self.slider_S_load = ttk.Scale(frame_load, from_=0, to=10000, orient='horizontal', length=200)
        self.slider_S_load.set(5000)
        self.slider_S_load.grid(row=0, column=1, pady=5)
        self.label_S_load = ttk.Label(frame_load, text="5000 kVA")
        self.label_S_load.grid(row=0, column=2, pady=5)
        self.slider_S_load.configure(command=lambda v: self.label_S_load.config(text=f"{float(v):.0f} kVA"))

        ttk.Label(frame_load, text="Power Factor:").grid(row=1, column=0, sticky='w', pady=5)
        self.slider_pf = ttk.Scale(frame_load, from_=0.5, to=1.0, orient='horizontal', length=200)
        self.slider_pf.set(0.8)
        self.slider_pf.grid(row=1, column=1, pady=5)
        self.label_pf = ttk.Label(frame_load, text="0.80")
        self.label_pf.grid(row=1, column=2, pady=5)
        self.slider_pf.configure(command=lambda v: self.label_pf.config(text=f"{float(v):.2f}"))

        ttk.Label(frame_load, text="Voltage (kV):").grid(row=2, column=0, sticky='w', pady=5)
        self.entry_voltage = ttk.Entry(frame_load)
        self.entry_voltage.insert(0, "11.0")
        self.entry_voltage.grid(row=2, column=1, pady=5)

        # Environmental parameters
        frame_env = ttk.LabelFrame(scrollable_frame, text="Environmental Conditions", padding=10)
        frame_env.grid(row=3, column=0, padx=10, pady=10, sticky='ew')

        ttk.Label(frame_env, text="Ambient Temperature (°C):").grid(row=0, column=0, sticky='w', pady=5)
        self.slider_ambient = ttk.Scale(frame_env, from_=-10, to=50, orient='horizontal', length=200)
        self.slider_ambient.set(25)
        self.slider_ambient.grid(row=0, column=1, pady=5)
        self.label_ambient = ttk.Label(frame_env, text="25°C")
        self.label_ambient.grid(row=0, column=2, pady=5)
        self.slider_ambient.configure(command=lambda v: self.label_ambient.config(text=f"{float(v):.0f}°C"))

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def setup_results_tab(self):
        """Setup results display tab"""
        # Results frame
        results_frame = ttk.Frame(self.tab_results, padding=10)
        results_frame.pack(fill='both', expand=True)

        # Left panel - text results
        left_frame = ttk.Frame(results_frame)
        left_frame.pack(side='left', fill='both', expand=True, padx=5)

        self.results_text = tk.Text(left_frame, height=30, width=50, font=('Courier', 10))
        self.results_text.pack(fill='both', expand=True)

        scrollbar = ttk.Scrollbar(left_frame, command=self.results_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Right panel - visualization
        right_frame = ttk.Frame(results_frame)
        right_frame.pack(side='right', fill='both', expand=True, padx=5)

        self.fig_results = Figure(figsize=(6, 6), dpi=100)
        self.canvas_results = FigureCanvasTkAgg(self.fig_results, right_frame)
        self.canvas_results.get_tk_widget().pack(fill='both', expand=True)

    def setup_dynamics_tab(self):
        """Setup dynamic simulation tab"""
        control_frame = ttk.Frame(self.tab_dynamics, padding=10)
        control_frame.pack(side='top', fill='x')

        ttk.Label(control_frame, text="ODE Solver:").pack(side='left', padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(control_frame, text="RK45 (Runge-Kutta)", variable=self.solver_var,
                       value='RK45').pack(side='left', padx=5)
        ttk.Radiobutton(control_frame, text="Euler", variable=self.solver_var,
                       value='Euler').pack(side='left', padx=5)

        ttk.Label(control_frame, text="Duration (s):").pack(side='left', padx=5)
        self.entry_duration = ttk.Entry(control_frame, width=10)
        self.entry_duration.insert(0, "3600")
        self.entry_duration.pack(side='left', padx=5)

        ttk.Button(control_frame, text="Run Simulation",
                  command=self.run_dynamic_simulation).pack(side='left', padx=5)

        # Graph area
        self.fig_dynamics = Figure(figsize=(10, 6), dpi=100)
        self.canvas_dynamics = FigureCanvasTkAgg(self.fig_dynamics, self.tab_dynamics)
        self.canvas_dynamics.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def setup_thermal_tab(self):
        """Setup thermal analysis tab"""
        # Graph area
        self.fig_thermal = Figure(figsize=(10, 6), dpi=100)
        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, self.tab_thermal)
        self.canvas_thermal.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def setup_economics_tab(self):
        """Setup economic analysis tab"""
        # Control frame
        control_frame = ttk.LabelFrame(self.tab_economics, text="Economic Parameters", padding=10)
        control_frame.pack(side='top', fill='x', padx=10, pady=10)

        ttk.Label(control_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', pady=5)
        self.entry_elec_cost = ttk.Entry(control_frame)
        self.entry_elec_cost.insert(0, "0.12")
        self.entry_elec_cost.grid(row=0, column=1, pady=5)

        ttk.Label(control_frame, text="Annual Maintenance ($):").grid(row=1, column=0, sticky='w', pady=5)
        self.entry_maint_cost = ttk.Entry(control_frame)
        self.entry_maint_cost.insert(0, "5000")
        self.entry_maint_cost.grid(row=1, column=1, pady=5)

        ttk.Label(control_frame, text="Analysis Period (years):").grid(row=2, column=0, sticky='w', pady=5)
        self.entry_years = ttk.Entry(control_frame)
        self.entry_years.insert(0, "10")
        self.entry_years.grid(row=2, column=1, pady=5)

        ttk.Button(control_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=3, column=0, columnspan=2, pady=10)

        # Results area
        self.economics_text = tk.Text(self.tab_economics, height=20, font=('Courier', 10))
        self.economics_text.pack(fill='both', expand=True, padx=10, pady=10)

    def setup_losses_tab(self):
        """Setup loss breakdown tab"""
        # Graph area for loss breakdown
        self.fig_losses = Figure(figsize=(10, 6), dpi=100)
        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, self.tab_losses)
        self.canvas_losses.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

    def setup_control_panel(self):
        """Setup main control panel"""
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side='bottom', fill='x', padx=5, pady=5)

        ttk.Button(control_frame, text="START", command=self.start_simulation,
                  style='Success.TButton').pack(side='left', padx=5)
        ttk.Button(control_frame, text="STOP", command=self.stop_simulation,
                  style='Danger.TButton').pack(side='left', padx=5)
        ttk.Button(control_frame, text="RESET", command=self.reset_simulation,
                  style='Warning.TButton').pack(side='left', padx=5)

        self.status_label = ttk.Label(control_frame, text="Status: Ready",
                                      font=('Arial', 10, 'bold'))
        self.status_label.pack(side='right', padx=5)

    def update_parameters(self):
        """Update transformer parameters from GUI inputs"""
        try:
            self.transformer.S_A_rated = float(self.entry_S_A.get())
            self.transformer.S_B_rated = float(self.entry_S_B.get())
            self.transformer.R_A_percent = float(self.slider_R_A.get())
            self.transformer.R_B_percent = float(self.slider_R_B.get())
            self.transformer.X_A_percent = float(self.slider_X_A.get())
            self.transformer.X_B_percent = float(self.slider_X_B.get())
            self.transformer.S_load = float(self.slider_S_load.get())
            self.transformer.pf = float(self.slider_pf.get())
            self.transformer.voltage = float(self.entry_voltage.get())
            self.transformer.ambient_temp = float(self.slider_ambient.get())
            return True
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input: {e}")
            return False

    def start_simulation(self):
        """Start the simulation"""
        if not self.update_parameters():
            return

        self.is_running = True
        self.status_label.config(text="Status: Running")

        # Calculate results
        results = self.transformer.calculate_load_distribution()

        # Display results
        self.display_results(results)

        # Update visualizations
        self.update_results_visualization(results)
        self.update_losses_visualization(results)

        self.status_label.config(text="Status: Completed")
        self.is_running = False

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_running = False
        self.status_label.config(text="Status: Stopped")

    def reset_simulation(self):
        """Reset the simulation"""
        self.is_running = False
        self.results_text.delete(1.0, tk.END)
        self.economics_text.delete(1.0, tk.END)

        # Clear all graphs
        self.fig_results.clear()
        self.canvas_results.draw()
        self.fig_dynamics.clear()
        self.canvas_dynamics.draw()
        self.fig_thermal.clear()
        self.canvas_thermal.draw()
        self.fig_losses.clear()
        self.canvas_losses.draw()

        self.status_label.config(text="Status: Reset")

    def display_results(self, results):
        """Display calculation results"""
        self.results_text.delete(1.0, tk.END)

        output = "="*60 + "\n"
        output += "TRANSFORMER PARALLEL LOAD DISTRIBUTION ANALYSIS\n"
        output += "="*60 + "\n\n"

        output += "INPUT PARAMETERS:\n"
        output += "-"*60 + "\n"
        output += f"Transformer A: {self.transformer.S_A_rated} kVA, R={self.transformer.R_A_percent}%, X={self.transformer.X_A_percent}%\n"
        output += f"Transformer B: {self.transformer.S_B_rated} kVA, R={self.transformer.R_B_percent}%, X={self.transformer.X_B_percent}%\n"
        output += f"Total Load: {self.transformer.S_load} kVA at {self.transformer.pf} p.f. lagging\n"
        output += f"System Voltage: {self.transformer.voltage} kV\n\n"

        output += "LOAD DISTRIBUTION RESULTS:\n"
        output += "-"*60 + "\n"
        output += f"Transformer A supplies: {results['S_A']:.2f} kVA ({results['loading_A']:.1f}% loading)\n"
        output += f"Transformer B supplies: {results['S_B']:.2f} kVA ({results['loading_B']:.1f}% loading)\n"
        output += f"Total supplied: {results['S_A'] + results['S_B']:.2f} kVA\n\n"

        output += f"Transformer A current: {results['I_A']:.2f} A\n"
        output += f"Transformer B current: {results['I_B']:.2f} A\n\n"

        output += "IMPEDANCE ANALYSIS:\n"
        output += "-"*60 + "\n"
        output += f"Z_A (common base): {results['Z_A']:.4f} pu\n"
        output += f"Z_B (common base): {results['Z_B']:.4f} pu\n"
        output += f"|Z_A|: {abs(results['Z_A']):.4f} pu\n"
        output += f"|Z_B|: {abs(results['Z_B']):.4f} pu\n\n"

        # Calculate losses
        losses_A = self.transformer.calculate_losses(
            results['S_A'], self.transformer.S_A_rated, self.transformer.R_A_percent
        )
        losses_B = self.transformer.calculate_losses(
            results['S_B'], self.transformer.S_B_rated, self.transformer.R_B_percent
        )

        output += "LOSS BREAKDOWN:\n"
        output += "-"*60 + "\n"
        output += "Transformer A:\n"
        output += f"  Copper losses:     {losses_A['copper']:.2f} kW\n"
        output += f"  Iron losses:       {losses_A['iron']:.2f} kW\n"
        output += f"  Stray losses:      {losses_A['stray']:.2f} kW\n"
        output += f"  Mechanical losses: {losses_A['mechanical']:.2f} kW\n"
        output += f"  Total losses:      {losses_A['total']:.2f} kW\n\n"

        output += "Transformer B:\n"
        output += f"  Copper losses:     {losses_B['copper']:.2f} kW\n"
        output += f"  Iron losses:       {losses_B['iron']:.2f} kW\n"
        output += f"  Stray losses:      {losses_B['stray']:.2f} kW\n"
        output += f"  Mechanical losses: {losses_B['mechanical']:.2f} kW\n"
        output += f"  Total losses:      {losses_B['total']:.2f} kW\n\n"

        output += f"Combined total losses: {losses_A['total'] + losses_B['total']:.2f} kW\n\n"

        # Calculate efficiency
        eff_A = self.transformer.calculate_efficiency(results['S_A'], losses_A)
        eff_B = self.transformer.calculate_efficiency(results['S_B'], losses_B)

        output += "EFFICIENCY:\n"
        output += "-"*60 + "\n"
        output += f"Transformer A: {eff_A:.2f}%\n"
        output += f"Transformer B: {eff_B:.2f}%\n\n"

        # Temperature estimation
        temp_A = self.transformer.calculate_temperature(results['S_A'], self.transformer.S_A_rated)
        temp_B = self.transformer.calculate_temperature(results['S_B'], self.transformer.S_B_rated)

        output += "THERMAL ANALYSIS (Steady-State):\n"
        output += "-"*60 + "\n"
        output += f"Ambient temperature: {self.transformer.ambient_temp}°C\n"
        output += f"Transformer A temperature: {temp_A:.1f}°C\n"
        output += f"Transformer B temperature: {temp_B:.1f}°C\n\n"

        # Derating check
        max_temp = 85  # °C
        output += "DERATING ANALYSIS:\n"
        output += "-"*60 + "\n"
        if temp_A > max_temp:
            derating_A = (max_temp / temp_A) * 100
            output += f"WARNING: Transformer A exceeds max temperature!\n"
            output += f"  Recommended derating: {100-derating_A:.1f}%\n"
        else:
            output += f"Transformer A: Within thermal limits\n"

        if temp_B > max_temp:
            derating_B = (max_temp / temp_B) * 100
            output += f"WARNING: Transformer B exceeds max temperature!\n"
            output += f"  Recommended derating: {100-derating_B:.1f}%\n"
        else:
            output += f"Transformer B: Within thermal limits\n"

        output += "\n" + "="*60 + "\n"
        output += f"Analysis completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        output += "="*60 + "\n"

        self.results_text.insert(1.0, output)

    def update_results_visualization(self, results):
        """Update results visualization"""
        self.fig_results.clear()

        # Create subplots
        ax1 = self.fig_results.add_subplot(221)
        ax2 = self.fig_results.add_subplot(222)
        ax3 = self.fig_results.add_subplot(223)
        ax4 = self.fig_results.add_subplot(224)

        # 1. Load distribution bar chart
        transformers = ['Transformer A', 'Transformer B']
        loads = [results['S_A'], results['S_B']]
        colors = ['#3498db', '#e74c3c']

        ax1.bar(transformers, loads, color=colors, alpha=0.7, edgecolor='black')
        ax1.set_ylabel('Load (kVA)')
        ax1.set_title('Load Distribution')
        ax1.grid(True, alpha=0.3)

        # Add value labels on bars
        for i, v in enumerate(loads):
            ax1.text(i, v + 100, f'{v:.0f} kVA', ha='center', fontweight='bold')

        # 2. Loading percentage
        loadings = [results['loading_A'], results['loading_B']]
        ax2.bar(transformers, loadings, color=colors, alpha=0.7, edgecolor='black')
        ax2.axhline(y=100, color='r', linestyle='--', label='Rated Load')
        ax2.set_ylabel('Loading (%)')
        ax2.set_title('Transformer Loading')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Add value labels
        for i, v in enumerate(loadings):
            ax2.text(i, v + 2, f'{v:.1f}%', ha='center', fontweight='bold')

        # 3. Impedance magnitude comparison
        impedances = [abs(results['Z_A']), abs(results['Z_B'])]
        ax3.bar(transformers, impedances, color=colors, alpha=0.7, edgecolor='black')
        ax3.set_ylabel('Impedance (pu)')
        ax3.set_title('Impedance Magnitude (Common Base)')
        ax3.grid(True, alpha=0.3)

        # 4. Current distribution
        currents = [results['I_A'], results['I_B']]
        ax4.bar(transformers, currents, color=colors, alpha=0.7, edgecolor='black')
        ax4.set_ylabel('Current (A)')
        ax4.set_title('Current Distribution')
        ax4.grid(True, alpha=0.3)

        for i, v in enumerate(currents):
            ax4.text(i, v + 10, f'{v:.0f} A', ha='center', fontweight='bold')

        self.fig_results.tight_layout()
        self.canvas_results.draw()

    def update_losses_visualization(self, results):
        """Update loss breakdown visualization"""
        self.fig_losses.clear()

        losses_A = self.transformer.calculate_losses(
            results['S_A'], self.transformer.S_A_rated, self.transformer.R_A_percent
        )
        losses_B = self.transformer.calculate_losses(
            results['S_B'], self.transformer.S_B_rated, self.transformer.R_B_percent
        )

        # Create subplots
        ax1 = self.fig_losses.add_subplot(221)
        ax2 = self.fig_losses.add_subplot(222)
        ax3 = self.fig_losses.add_subplot(223)
        ax4 = self.fig_losses.add_subplot(224)

        # 1. Transformer A loss breakdown (pie chart)
        labels_A = ['Copper', 'Iron', 'Stray', 'Mechanical']
        sizes_A = [losses_A['copper'], losses_A['iron'], losses_A['stray'], losses_A['mechanical']]
        colors_pie = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#96ceb4']

        ax1.pie(sizes_A, labels=labels_A, autopct='%1.1f%%', colors=colors_pie, startangle=90)
        ax1.set_title('Transformer A Loss Breakdown')

        # 2. Transformer B loss breakdown (pie chart)
        sizes_B = [losses_B['copper'], losses_B['iron'], losses_B['stray'], losses_B['mechanical']]
        ax2.pie(sizes_B, labels=labels_A, autopct='%1.1f%%', colors=colors_pie, startangle=90)
        ax2.set_title('Transformer B Loss Breakdown')

        # 3. Total losses comparison
        categories = ['Copper', 'Iron', 'Stray', 'Mechanical']
        x = np.arange(len(categories))
        width = 0.35

        losses_A_list = [losses_A['copper'], losses_A['iron'], losses_A['stray'], losses_A['mechanical']]
        losses_B_list = [losses_B['copper'], losses_B['iron'], losses_B['stray'], losses_B['mechanical']]

        ax3.bar(x - width/2, losses_A_list, width, label='Transformer A', color='#3498db', alpha=0.7)
        ax3.bar(x + width/2, losses_B_list, width, label='Transformer B', color='#e74c3c', alpha=0.7)
        ax3.set_ylabel('Loss (kW)')
        ax3.set_title('Loss Comparison by Category')
        ax3.set_xticks(x)
        ax3.set_xticklabels(categories, rotation=45)
        ax3.legend()
        ax3.grid(True, alpha=0.3, axis='y')

        # 4. Total losses bar chart
        total_losses = [losses_A['total'], losses_B['total']]
        transformers = ['Transformer A', 'Transformer B']
        ax4.bar(transformers, total_losses, color=['#3498db', '#e74c3c'], alpha=0.7, edgecolor='black')
        ax4.set_ylabel('Total Loss (kW)')
        ax4.set_title('Total Losses Comparison')
        ax4.grid(True, alpha=0.3, axis='y')

        for i, v in enumerate(total_losses):
            ax4.text(i, v + 0.5, f'{v:.2f} kW', ha='center', fontweight='bold')

        self.fig_losses.tight_layout()
        self.canvas_losses.draw()

    def run_dynamic_simulation(self):
        """Run dynamic thermal simulation"""
        if not self.update_parameters():
            return

        self.status_label.config(text="Status: Running Dynamic Simulation...")

        try:
            duration = float(self.entry_duration.get())
            method = self.solver_var.get()

            # Run simulation
            sim_results = self.simulator.simulate_dynamic_response(duration, method)

            # Plot results
            self.fig_dynamics.clear()

            ax1 = self.fig_dynamics.add_subplot(211)
            ax2 = self.fig_dynamics.add_subplot(212)

            # Temperature evolution
            ax1.plot(sim_results['time'], sim_results['temp_A'],
                    label='Transformer A', color='#3498db', linewidth=2)
            ax1.plot(sim_results['time'], sim_results['temp_B'],
                    label='Transformer B', color='#e74c3c', linewidth=2)
            ax1.axhline(y=85, color='r', linestyle='--', label='Max Temp Limit', alpha=0.7)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Temperature (°C)')
            ax1.set_title(f'Thermal Response - {method} Method')
            ax1.legend()
            ax1.grid(True, alpha=0.3)

            # Temperature difference
            temp_diff = sim_results['temp_A'] - sim_results['temp_B']
            ax2.plot(sim_results['time'], temp_diff, color='#9b59b6', linewidth=2)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Temperature Difference (°C)')
            ax2.set_title('Temperature Difference (A - B)')
            ax2.grid(True, alpha=0.3)
            ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)

            self.fig_dynamics.tight_layout()
            self.canvas_dynamics.draw()

            # Also update thermal tab
            self.update_thermal_visualization(sim_results)

            self.status_label.config(text="Status: Dynamic Simulation Completed")

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {e}")
            self.status_label.config(text="Status: Simulation Error")

    def update_thermal_visualization(self, sim_results):
        """Update thermal analysis visualization"""
        self.fig_thermal.clear()

        ax1 = self.fig_thermal.add_subplot(221)
        ax2 = self.fig_thermal.add_subplot(222)
        ax3 = self.fig_thermal.add_subplot(223)
        ax4 = self.fig_thermal.add_subplot(224)

        # 1. Temperature evolution
        ax1.plot(sim_results['time']/60, sim_results['temp_A'],
                label='Transformer A', color='#3498db', linewidth=2)
        ax1.plot(sim_results['time']/60, sim_results['temp_B'],
                label='Transformer B', color='#e74c3c', linewidth=2)
        ax1.axhline(y=85, color='r', linestyle='--', label='Limit', alpha=0.5)
        ax1.set_xlabel('Time (min)')
        ax1.set_ylabel('Temperature (°C)')
        ax1.set_title('Temperature Evolution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. Temperature rate of change
        dt = sim_results['time'][1] - sim_results['time'][0]
        dTdt_A = np.gradient(sim_results['temp_A'], dt)
        dTdt_B = np.gradient(sim_results['temp_B'], dt)

        ax2.plot(sim_results['time']/60, dTdt_A, label='Transformer A', color='#3498db', linewidth=2)
        ax2.plot(sim_results['time']/60, dTdt_B, label='Transformer B', color='#e74c3c', linewidth=2)
        ax2.set_xlabel('Time (min)')
        ax2.set_ylabel('dT/dt (°C/s)')
        ax2.set_title('Temperature Rate of Change')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # 3. Thermal time constant analysis
        # Exponential fit to estimate time constant
        final_temp_A = sim_results['temp_A'][-1]
        initial_temp = self.transformer.ambient_temp
        time_to_63_2_percent_A = None

        target_temp = initial_temp + 0.632 * (final_temp_A - initial_temp)
        for i, temp in enumerate(sim_results['temp_A']):
            if temp >= target_temp:
                time_to_63_2_percent_A = sim_results['time'][i]
                break

        ax3.plot(sim_results['time']/60, sim_results['temp_A'],
                color='#3498db', linewidth=2, label='Actual')
        if time_to_63_2_percent_A:
            ax3.axvline(x=time_to_63_2_percent_A/60, color='g', linestyle='--',
                       label=f'τ = {time_to_63_2_percent_A/60:.1f} min')
            ax3.axhline(y=target_temp, color='orange', linestyle='--', alpha=0.5)
        ax3.set_xlabel('Time (min)')
        ax3.set_ylabel('Temperature (°C)')
        ax3.set_title('Transformer A - Time Constant Analysis')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # 4. Comparative thermal performance
        max_temp_A = np.max(sim_results['temp_A'])
        max_temp_B = np.max(sim_results['temp_B'])
        avg_temp_A = np.mean(sim_results['temp_A'])
        avg_temp_B = np.mean(sim_results['temp_B'])

        categories = ['Max Temp', 'Avg Temp']
        x = np.arange(len(categories))
        width = 0.35

        values_A = [max_temp_A, avg_temp_A]
        values_B = [max_temp_B, avg_temp_B]

        ax4.bar(x - width/2, values_A, width, label='Transformer A', color='#3498db', alpha=0.7)
        ax4.bar(x + width/2, values_B, width, label='Transformer B', color='#e74c3c', alpha=0.7)
        ax4.set_ylabel('Temperature (°C)')
        ax4.set_title('Thermal Performance Comparison')
        ax4.set_xticks(x)
        ax4.set_xticklabels(categories)
        ax4.legend()
        ax4.grid(True, alpha=0.3, axis='y')

        self.fig_thermal.tight_layout()
        self.canvas_thermal.draw()

    def calculate_economics(self):
        """Calculate and display economic analysis"""
        if not self.update_parameters():
            return

        try:
            # Update economic parameters
            self.economics.electricity_cost = float(self.entry_elec_cost.get())
            self.economics.maintenance_cost_per_year = float(self.entry_maint_cost.get())
            years = int(self.entry_years.get())

            # Calculate
            operating_cost = self.economics.calculate_operating_cost()
            roi_analysis = self.economics.calculate_roi(years)

            # Display results
            self.economics_text.delete(1.0, tk.END)

            output = "="*70 + "\n"
            output += "ECONOMIC ANALYSIS\n"
            output += "="*70 + "\n\n"

            output += "OPERATING COSTS (Annual):\n"
            output += "-"*70 + "\n"
            output += f"Total Energy Loss:        {operating_cost['energy_loss_kWh']:,.0f} kWh/year\n"
            output += f"Energy Cost:              ${operating_cost['energy_cost']:,.2f}/year\n"
            output += f"Maintenance Cost:         ${operating_cost['maintenance_cost']:,.2f}/year\n"
            output += f"Total Operating Cost:     ${operating_cost['total_operating_cost']:,.2f}/year\n\n"

            output += "TRANSFORMER A LOSSES:\n"
            output += "-"*70 + "\n"
            for loss_type, value in operating_cost['losses_A'].items():
                output += f"  {loss_type.capitalize():20s}: {value:.2f} kW\n"

            output += "\nTRANSFORMER B LOSSES:\n"
            output += "-"*70 + "\n"
            for loss_type, value in operating_cost['losses_B'].items():
                output += f"  {loss_type.capitalize():20s}: {value:.2f} kW\n"

            output += f"\nINVESTMENT ANALYSIS ({years} years):\n"
            output += "-"*70 + "\n"
            output += f"Initial Investment:       ${self.economics.transformer_A_cost + self.economics.transformer_B_cost:,.2f}\n"
            output += f"Total Operating Cost:     ${roi_analysis['total_cost']:,.2f}\n"
            output += f"Total Revenue:            ${roi_analysis['total_revenue']:,.2f}\n"
            output += f"Return on Investment:     {roi_analysis['roi']:.2f}%\n"
            output += f"Payback Period:           {roi_analysis['payback_period']:.2f} years\n\n"

            # Cost per kWh delivered
            total_energy_delivered = self.transformer.S_load * self.transformer.pf * 8760 * years
            cost_per_kwh = roi_analysis['total_cost'] / total_energy_delivered
            output += f"EFFICIENCY METRICS:\n"
            output += "-"*70 + "\n"
            output += f"Cost per kWh delivered:   ${cost_per_kwh:.4f}/kWh\n"
            output += f"Loss percentage:          {(operating_cost['energy_loss_kWh']/(self.transformer.S_load*self.transformer.pf*8760))*100:.2f}%\n"

            output += "\n" + "="*70 + "\n"

            self.economics_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Calculation Error", f"Error in economic analysis: {e}")

    def on_window_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # This will automatically handle canvas resizing
        if hasattr(self, 'canvas_results'):
            self.canvas_results.draw_idle()
        if hasattr(self, 'canvas_dynamics'):
            self.canvas_dynamics.draw_idle()
        if hasattr(self, 'canvas_thermal'):
            self.canvas_thermal.draw_idle()
        if hasattr(self, 'canvas_losses'):
            self.canvas_losses.draw_idle()

    def save_results(self):
        """Save results to file"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"transformer_analysis_{timestamp}.txt"

            with open(filename, 'w') as f:
                f.write(self.results_text.get(1.0, tk.END))
                f.write("\n\n" + "="*70 + "\n")
                f.write("ECONOMIC ANALYSIS\n")
                f.write("="*70 + "\n")
                f.write(self.economics_text.get(1.0, tk.END))

            messagebox.showinfo("Success", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving results: {e}")

    def load_config(self):
        """Load configuration (placeholder)"""
        messagebox.showinfo("Info", "Configuration loading not yet implemented")

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced Transformer Load Distribution Simulator
Version 1.0

Features:
• Parallel transformer load distribution analysis
• Multi-physics simulation (electromagnetic, thermal, mechanical)
• Real-time ODE solvers (RK45, Euler)
• Dynamic visualization and economic analysis
• Thermal derating and advanced controls
• Detailed loss breakdown

Developed for electrical engineering education and practical applications.
"""
        messagebox.showinfo("About", about_text)

    def show_docs(self):
        """Show documentation (placeholder)"""
        docs_text = """
QUICK START GUIDE:

1. Input Parameters Tab:
   - Adjust transformer ratings and parameters using sliders
   - Set load conditions and environmental factors

2. Start Simulation:
   - Click START button to run calculations
   - View results in Load Distribution tab

3. Dynamic Simulation:
   - Navigate to Dynamic Simulation tab
   - Select ODE solver method (RK45 or Euler)
   - Set simulation duration and click Run Simulation

4. Economic Analysis:
   - Go to Economic Analysis tab
   - Enter cost parameters
   - Click Calculate Economics

5. View Results:
   - Check Loss Breakdown tab for detailed loss analysis
   - Thermal Analysis tab shows temperature profiles
"""
        messagebox.showinfo("Documentation", docs_text)


def main():
    """Main application entry point"""
    root = tk.Tk()

    # Configure style
    style = ttk.Style()
    style.theme_use('clam')

    # Create and run application
    app = AdvancedTransformerGUI(root)

    # Start main loop
    root.mainloop()


if __name__ == "__main__":
    main()
