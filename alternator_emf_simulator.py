#!/usr/bin/env python3
"""
Advanced Alternator and Electrical Machine Simulator
Multi-Physics Simulation with Electromagnetic, Thermal, and Mechanical Analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
import threading
import time
from dataclasses import dataclass
from typing import Tuple, List, Dict
import json


@dataclass
class AlternatorParameters:
    """Alternator physical parameters"""
    poles: int = 4
    frequency: float = 50.0
    slots_per_pole: int = 15
    conductors_per_slot: int = 10
    winding_factor: float = 0.95
    connection_type: str = "star"  # star or delta
    flux_per_pole: float = 0.05  # Weber
    speed_rpm: float = 1500.0

    # Electrical parameters
    resistance_per_phase: float = 0.5  # Ohms
    inductance_per_phase: float = 0.01  # Henry
    load_resistance: float = 100.0  # Ohms
    load_inductance: float = 0.05  # Henry

    # Thermal parameters
    ambient_temp: float = 25.0  # Celsius
    thermal_resistance: float = 2.0  # K/W
    thermal_capacitance: float = 500.0  # J/K
    max_temp: float = 155.0  # Celsius (Class F insulation)

    # Mechanical parameters
    moment_of_inertia: float = 0.5  # kg·m²
    friction_coefficient: float = 0.01  # N·m·s
    shaft_stiffness: float = 1e6  # N·m/rad

    # Economic parameters
    energy_cost: float = 0.12  # $/kWh
    maintenance_cost_per_hour: float = 5.0  # $/hour
    initial_cost: float = 10000.0  # $


class AlternatorCalculator:
    """Core calculation module for alternator analysis"""

    def __init__(self, params: AlternatorParameters):
        self.params = params
        self.reset_simulation()

    def reset_simulation(self):
        """Reset simulation state"""
        self.time_history = []
        self.voltage_history = []
        self.current_history = []
        self.power_history = []
        self.temperature_history = []
        self.speed_history = []
        self.torque_history = []
        self.efficiency_history = []
        self.losses_history = {'copper': [], 'iron': [], 'mechanical': [], 'stray': []}

    @property
    def total_slots(self) -> int:
        """Calculate total number of slots"""
        return self.params.poles * self.params.slots_per_pole

    @property
    def total_conductors(self) -> int:
        """Calculate total number of conductors"""
        return self.total_slots * self.params.conductors_per_slot

    @property
    def conductors_per_phase(self) -> int:
        """Calculate conductors per phase"""
        return self.total_conductors // 3

    @property
    def turns_per_phase(self) -> int:
        """Calculate turns per phase"""
        return self.conductors_per_phase // 2

    def calculate_star_emf(self) -> Tuple[float, float]:
        """Calculate EMF for star-connected alternator
        Returns: (phase_voltage, line_voltage) in Volts RMS
        """
        # EMF per phase: Eph = 4.44 × f × φ × Tph × Kw
        eph = (4.44 * self.params.frequency * self.params.flux_per_pole *
               self.turns_per_phase * self.params.winding_factor)

        # Line voltage for star connection
        line_voltage = np.sqrt(3) * eph

        return eph, line_voltage

    def calculate_delta_emf(self) -> Tuple[float, float]:
        """Calculate EMF for delta-connected alternator
        Returns: (phase_voltage, line_voltage) in Volts RMS
        """
        # EMF per phase: Eph = 4.44 × f × φ × Tph × Kw
        eph = (4.44 * self.params.frequency * self.params.flux_per_pole *
               self.turns_per_phase * self.params.winding_factor)

        # Line voltage for delta connection equals phase voltage
        line_voltage = eph

        return eph, line_voltage

    def calculate_lap_winding_emf(self) -> float:
        """Calculate EMF for lap-connected winding (DC machine style)
        Returns: EMF in Volts
        """
        # For lap winding: A = P (number of parallel paths)
        A = self.params.poles
        Z = self.total_conductors
        n = self.params.speed_rpm
        phi = self.params.flux_per_pole
        P = self.params.poles

        # E = (φ × Z × n × P) / (60 × A)
        emf = (phi * Z * n * P) / (60 * A)

        return emf

    def calculate_wave_winding_emf(self) -> float:
        """Calculate EMF for wave-connected winding (DC machine style)
        Returns: EMF in Volts
        """
        # For wave winding: A = 2 (always 2 parallel paths)
        A = 2
        Z = self.total_conductors
        n = self.params.speed_rpm
        phi = self.params.flux_per_pole
        P = self.params.poles

        # E = (φ × Z × n × P) / (60 × A)
        emf = (phi * Z * n * P) / (60 * A)

        return emf

    def calculate_losses(self, current_rms: float, speed_rpm: float) -> Dict[str, float]:
        """Calculate detailed losses
        Returns: Dictionary of losses in Watts
        """
        # Copper losses (I²R losses)
        copper_loss = 3 * (current_rms ** 2) * self.params.resistance_per_phase

        # Iron losses (hysteresis + eddy current)
        # Hysteresis loss: Ph = Kh × f × Bmax^1.6 × Volume
        # Eddy current loss: Pe = Ke × f² × Bmax² × Volume
        # Simplified model:
        kh = 0.01  # Hysteresis constant
        ke = 0.001  # Eddy current constant
        f = self.params.frequency * (speed_rpm / self.params.speed_rpm)
        Bmax = self.params.flux_per_pole / 0.01  # Simplified flux density

        hysteresis_loss = kh * f * (Bmax ** 1.6)
        eddy_loss = ke * (f ** 2) * (Bmax ** 2)
        iron_loss = hysteresis_loss + eddy_loss

        # Mechanical losses (friction + windage)
        mechanical_loss = self.params.friction_coefficient * (speed_rpm / 60) ** 2

        # Stray load losses (typically 1% of output power)
        eph, _ = self.calculate_star_emf()
        output_power = 3 * eph * current_rms * 0.8  # Assume 0.8 power factor
        stray_loss = 0.01 * output_power

        return {
            'copper': copper_loss,
            'iron': iron_loss,
            'mechanical': mechanical_loss,
            'stray': stray_loss,
            'total': copper_loss + iron_loss + mechanical_loss + stray_loss
        }

    def calculate_efficiency(self, output_power: float, losses: Dict[str, float]) -> float:
        """Calculate efficiency
        Returns: Efficiency as percentage
        """
        total_loss = losses['total']
        input_power = output_power + total_loss

        if input_power == 0:
            return 0.0

        efficiency = (output_power / input_power) * 100
        return min(efficiency, 100.0)

    def electromagnetic_thermal_ode(self, t: float, y: np.ndarray,
                                   load_type: str = 'RL') -> np.ndarray:
        """Coupled electromagnetic-thermal differential equations

        State vector y = [i_a, i_b, i_c, theta, omega, temperature]
        where:
            i_a, i_b, i_c: phase currents (A)
            theta: rotor angle (rad)
            omega: angular velocity (rad/s)
            temperature: winding temperature (°C)

        Returns: dy/dt
        """
        i_a, i_b, i_c, theta, omega, temp = y

        # Electrical parameters (temperature dependent)
        R_temp_coeff = 0.00393  # Copper temperature coefficient
        R = self.params.resistance_per_phase * (1 + R_temp_coeff * (temp - 25))
        L = self.params.inductance_per_phase

        # Back EMF calculation
        omega_elec = omega * self.params.poles / 2
        emf_peak = np.sqrt(2) * 4.44 * (omega / (2 * np.pi)) * self.params.flux_per_pole * \
                   self.turns_per_phase * self.params.winding_factor

        e_a = emf_peak * np.sin(omega_elec * t)
        e_b = emf_peak * np.sin(omega_elec * t - 2 * np.pi / 3)
        e_c = emf_peak * np.sin(omega_elec * t + 2 * np.pi / 3)

        # Load impedance
        R_load = self.params.load_resistance
        L_load = self.params.load_inductance

        # Voltage equations (KVL)
        if load_type == 'RL':
            di_a_dt = (e_a - (R + R_load) * i_a) / (L + L_load)
            di_b_dt = (e_b - (R + R_load) * i_b) / (L + L_load)
            di_c_dt = (e_c - (R + R_load) * i_c) / (L + L_load)
        else:  # Resistive load
            di_a_dt = (e_a - (R + R_load) * i_a) / L
            di_b_dt = (e_b - (R + R_load) * i_b) / L
            di_c_dt = (e_c - (R + R_load) * i_c) / L

        # Mechanical equation
        # Electromagnetic torque
        Te = (e_a * i_a + e_b * i_b + e_c * i_c) / omega if omega != 0 else 0

        # Load torque (assume constant for now)
        Tload = 0.8 * Te  # 80% of electromagnetic torque

        # Mechanical friction
        Tfriction = self.params.friction_coefficient * omega

        # Angular acceleration
        J = self.params.moment_of_inertia
        domega_dt = (Te - Tload - Tfriction) / J

        # Angular position
        dtheta_dt = omega

        # Thermal equation
        # Heat generation (total losses)
        copper_loss = R * (i_a**2 + i_b**2 + i_c**2)
        iron_loss = 0.01 * omega**2  # Simplified model
        total_heat = copper_loss + iron_loss

        # Heat dissipation
        Rth = self.params.thermal_resistance
        Cth = self.params.thermal_capacitance
        T_amb = self.params.ambient_temp

        dtemp_dt = (total_heat - (temp - T_amb) / Rth) / Cth

        return np.array([di_a_dt, di_b_dt, di_c_dt, dtheta_dt, domega_dt, dtemp_dt])

    def simulate_dynamic(self, duration: float, dt: float, method: str = 'RK45',
                        callback=None) -> Dict[str, np.ndarray]:
        """Dynamic simulation using ODE solver

        Args:
            duration: Simulation duration (seconds)
            dt: Time step (seconds)
            method: Integration method ('RK45', 'RK23', 'Euler', 'DOP853')
            callback: Optional callback function for real-time updates

        Returns: Dictionary with time series data
        """
        # Initial conditions [i_a, i_b, i_c, theta, omega, temperature]
        omega_0 = 2 * np.pi * self.params.speed_rpm / 60
        y0 = np.array([0.0, 0.0, 0.0, 0.0, omega_0, self.params.ambient_temp])

        # Time points
        t_span = (0, duration)
        t_eval = np.arange(0, duration, dt)

        if method.upper() == 'EULER':
            # Euler method implementation
            solution = self._solve_euler(y0, t_eval, callback)
        else:
            # Use scipy's solve_ivp
            solution = solve_ivp(
                self.electromagnetic_thermal_ode,
                t_span,
                y0,
                method=method,
                t_eval=t_eval,
                max_step=dt,
                vectorized=False
            )

            if callback:
                for i in range(len(solution.t)):
                    callback(i, len(solution.t))

        # Extract results
        if method.upper() == 'EULER':
            t = solution['t']
            y = solution['y']
        else:
            t = solution.t
            y = solution.y

        # Calculate RMS values and power
        i_a, i_b, i_c = y[0], y[1], y[2]
        theta, omega, temp = y[3], y[4], y[5]

        # Calculate instantaneous voltage and power
        voltage_rms = np.zeros(len(t))
        current_rms = np.zeros(len(t))
        power = np.zeros(len(t))
        torque = np.zeros(len(t))

        for i in range(len(t)):
            # RMS current (three-phase)
            current_rms[i] = np.sqrt((i_a[i]**2 + i_b[i]**2 + i_c[i]**2) / 3)

            # RMS voltage
            omega_elec = omega[i] * self.params.poles / 2
            emf_peak = np.sqrt(2) * 4.44 * (omega[i] / (2 * np.pi)) * \
                      self.params.flux_per_pole * self.turns_per_phase * \
                      self.params.winding_factor
            voltage_rms[i] = emf_peak / np.sqrt(2)

            # Instantaneous power
            e_a = emf_peak * np.sin(omega_elec * t[i])
            e_b = emf_peak * np.sin(omega_elec * t[i] - 2 * np.pi / 3)
            e_c = emf_peak * np.sin(omega_elec * t[i] + 2 * np.pi / 3)
            power[i] = e_a * i_a[i] + e_b * i_b[i] + e_c * i_c[i]

            # Electromagnetic torque
            torque[i] = (e_a * i_a[i] + e_b * i_b[i] + e_c * i_c[i]) / omega[i] if omega[i] != 0 else 0

        return {
            'time': t,
            'current_a': i_a,
            'current_b': i_b,
            'current_c': i_c,
            'current_rms': current_rms,
            'voltage_rms': voltage_rms,
            'power': power,
            'torque': torque,
            'theta': theta,
            'omega': omega,
            'speed_rpm': omega * 60 / (2 * np.pi),
            'temperature': temp
        }

    def _solve_euler(self, y0: np.ndarray, t_eval: np.ndarray,
                     callback=None) -> Dict[str, np.ndarray]:
        """Euler method implementation"""
        y = np.zeros((len(y0), len(t_eval)))
        y[:, 0] = y0

        for i in range(1, len(t_eval)):
            dt = t_eval[i] - t_eval[i-1]
            dydt = self.electromagnetic_thermal_ode(t_eval[i-1], y[:, i-1])
            y[:, i] = y[:, i-1] + dt * dydt

            if callback:
                callback(i, len(t_eval))

        return {'t': t_eval, 'y': y}


class EconomicAnalyzer:
    """Economic analysis module"""

    def __init__(self, params: AlternatorParameters):
        self.params = params

    def calculate_operating_cost(self, power_kw: float, hours: float) -> Dict[str, float]:
        """Calculate operating costs"""
        energy_cost = power_kw * hours * self.params.energy_cost
        maintenance_cost = hours * self.params.maintenance_cost_per_hour
        total_cost = energy_cost + maintenance_cost

        return {
            'energy_cost': energy_cost,
            'maintenance_cost': maintenance_cost,
            'total_cost': total_cost,
            'cost_per_kwh': total_cost / (power_kw * hours) if (power_kw * hours) > 0 else 0
        }

    def calculate_lifecycle_cost(self, lifetime_years: float,
                                 avg_power_kw: float,
                                 hours_per_year: float) -> Dict[str, float]:
        """Calculate lifecycle costs"""
        total_hours = lifetime_years * hours_per_year
        operating_costs = self.calculate_operating_cost(avg_power_kw, total_hours)

        total_lifecycle_cost = self.params.initial_cost + operating_costs['total_cost']

        return {
            'initial_cost': self.params.initial_cost,
            'total_operating_cost': operating_costs['total_cost'],
            'total_lifecycle_cost': total_lifecycle_cost,
            'levelized_cost_per_kwh': total_lifecycle_cost / (avg_power_kw * total_hours)
        }

    def calculate_payback_period(self, annual_revenue: float) -> float:
        """Calculate simple payback period in years"""
        if annual_revenue <= 0:
            return float('inf')
        return self.params.initial_cost / annual_revenue


class AlternatorSimulatorGUI:
    """Main GUI application"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Alternator & Electrical Machine Simulator - Multi-Physics Analysis")
        self.root.geometry("1400x900")

        # Initialize parameters and calculator
        self.params = AlternatorParameters()
        self.calculator = AlternatorCalculator(self.params)
        self.economic_analyzer = EconomicAnalyzer(self.params)

        # Simulation control
        self.simulation_running = False
        self.simulation_thread = None

        # Create GUI
        self.create_menu()
        self.create_tabs()

        # Bind resize event for auto-scaling
        self.root.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.update_calculations()

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Configuration", command=self.save_configuration)
        file_menu.add_command(label="Load Configuration", command=self.load_configuration)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Documentation", command=self.show_documentation)

    def create_tabs(self):
        """Create tabbed interface"""
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_dynamic_simulation_tab()
        self.create_multi_physics_tab()
        self.create_losses_analysis_tab()
        self.create_economic_analysis_tab()
        self.create_advanced_controls_tab()

    def create_main_tab(self):
        """Main calculation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Main Calculations")

        # Left panel - Input parameters
        left_frame = ttk.LabelFrame(tab, text="Input Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create input fields with sliders
        self.create_parameter_input(left_frame, "Number of Poles:",
                                    lambda: self.params.poles,
                                    lambda v: setattr(self.params, 'poles', int(v)),
                                    2, 12, 1, row=0)

        self.create_parameter_input(left_frame, "Frequency (Hz):",
                                    lambda: self.params.frequency,
                                    lambda v: setattr(self.params, 'frequency', float(v)),
                                    50, 60, 0.1, row=1)

        self.create_parameter_input(left_frame, "Slots per Pole:",
                                    lambda: self.params.slots_per_pole,
                                    lambda v: setattr(self.params, 'slots_per_pole', int(v)),
                                    10, 20, 1, row=2)

        self.create_parameter_input(left_frame, "Conductors per Slot:",
                                    lambda: self.params.conductors_per_slot,
                                    lambda v: setattr(self.params, 'conductors_per_slot', int(v)),
                                    5, 20, 1, row=3)

        self.create_parameter_input(left_frame, "Winding Factor:",
                                    lambda: self.params.winding_factor,
                                    lambda v: setattr(self.params, 'winding_factor', float(v)),
                                    0.8, 1.0, 0.01, row=4)

        self.create_parameter_input(left_frame, "Flux per Pole (Wb):",
                                    lambda: self.params.flux_per_pole,
                                    lambda v: setattr(self.params, 'flux_per_pole', float(v)),
                                    0.01, 0.1, 0.001, row=5)

        self.create_parameter_input(left_frame, "Speed (RPM):",
                                    lambda: self.params.speed_rpm,
                                    lambda v: setattr(self.params, 'speed_rpm', float(v)),
                                    1000, 3000, 10, row=6)

        # Connection type
        ttk.Label(left_frame, text="Connection Type:").grid(row=7, column=0, sticky='w', pady=5)
        self.connection_var = tk.StringVar(value="star")
        ttk.Radiobutton(left_frame, text="Star", variable=self.connection_var,
                       value="star", command=self.update_calculations).grid(row=7, column=1, sticky='w')
        ttk.Radiobutton(left_frame, text="Delta", variable=self.connection_var,
                       value="delta", command=self.update_calculations).grid(row=7, column=2, sticky='w')

        # Calculate button
        ttk.Button(left_frame, text="Calculate", command=self.update_calculations,
                  style='Accent.TButton').grid(row=8, column=0, columnspan=3, pady=10)

        # Right panel - Results
        right_frame = ttk.LabelFrame(tab, text="Calculation Results", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Results text area
        self.results_text = scrolledtext.ScrolledText(right_frame, width=60, height=30,
                                                      font=('Courier', 10))
        self.results_text.pack(fill='both', expand=True)

        # Configure grid weights
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=2)
        tab.rowconfigure(0, weight=1)

    def create_parameter_input(self, parent, label, get_value, set_value,
                               min_val, max_val, step, row):
        """Create parameter input with label, entry, and slider"""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', pady=5)

        # Entry
        var = tk.DoubleVar(value=get_value())
        entry = ttk.Entry(parent, textvariable=var, width=10)
        entry.grid(row=row, column=1, padx=5)

        # Slider
        slider = ttk.Scale(parent, from_=min_val, to=max_val,
                          orient='horizontal', length=200,
                          command=lambda v: self.on_slider_change(var, set_value, float(v)))
        slider.set(get_value())
        slider.grid(row=row, column=2, padx=5)

        # Bind entry to update slider
        var.trace('w', lambda *args: self.on_entry_change(slider, var, set_value))

    def on_slider_change(self, var, set_value, value):
        """Handle slider change"""
        var.set(round(value, 3))
        set_value(value)
        self.update_calculations()

    def on_entry_change(self, slider, var, set_value):
        """Handle entry change"""
        try:
            value = var.get()
            slider.set(value)
            set_value(value)
            self.update_calculations()
        except:
            pass

    def create_dynamic_simulation_tab(self):
        """Dynamic simulation tab with ODE solvers"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.pack(side='top', fill='x', padx=5, pady=5)

        # Simulation parameters
        ttk.Label(control_frame, text="Duration (s):").grid(row=0, column=0, padx=5)
        self.sim_duration_var = tk.DoubleVar(value=1.0)
        ttk.Entry(control_frame, textvariable=self.sim_duration_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Time Step (s):").grid(row=0, column=2, padx=5)
        self.sim_dt_var = tk.DoubleVar(value=0.001)
        ttk.Entry(control_frame, textvariable=self.sim_dt_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=4, padx=5)
        self.sim_method_var = tk.StringVar(value="RK45")
        method_combo = ttk.Combobox(control_frame, textvariable=self.sim_method_var,
                                    values=['RK45', 'RK23', 'DOP853', 'Euler'],
                                    width=10, state='readonly')
        method_combo.grid(row=0, column=5, padx=5)

        # Control buttons
        self.start_btn = ttk.Button(control_frame, text="▶ Start",
                                    command=self.start_simulation)
        self.start_btn.grid(row=0, column=6, padx=5)

        self.stop_btn = ttk.Button(control_frame, text="⬛ Stop",
                                   command=self.stop_simulation, state='disabled')
        self.stop_btn.grid(row=0, column=7, padx=5)

        ttk.Button(control_frame, text="⟲ Reset",
                  command=self.reset_simulation).grid(row=0, column=8, padx=5)

        # Progress bar
        self.sim_progress = ttk.Progressbar(control_frame, mode='determinate', length=200)
        self.sim_progress.grid(row=1, column=0, columnspan=9, pady=10, sticky='ew')

        # Plots frame
        plots_frame = ttk.Frame(tab)
        plots_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figure with subplots
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, master=plots_frame)
        self.sim_canvas.get_tk_widget().pack(fill='both', expand=True)

        # Create subplots
        self.ax_voltage = self.sim_fig.add_subplot(3, 2, 1)
        self.ax_current = self.sim_fig.add_subplot(3, 2, 2)
        self.ax_power = self.sim_fig.add_subplot(3, 2, 3)
        self.ax_speed = self.sim_fig.add_subplot(3, 2, 4)
        self.ax_temp = self.sim_fig.add_subplot(3, 2, 5)
        self.ax_torque = self.sim_fig.add_subplot(3, 2, 6)

        self.sim_fig.tight_layout()

    def create_multi_physics_tab(self):
        """Multi-physics simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Multi-Physics Analysis")

        # Parameters frame
        params_frame = ttk.LabelFrame(tab, text="Multi-Physics Parameters", padding=10)
        params_frame.pack(side='top', fill='x', padx=5, pady=5)

        # Thermal parameters
        ttk.Label(params_frame, text="Thermal Resistance (K/W):").grid(row=0, column=0, sticky='w', padx=5, pady=3)
        self.thermal_r_var = tk.DoubleVar(value=self.params.thermal_resistance)
        ttk.Entry(params_frame, textvariable=self.thermal_r_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(params_frame, text="Thermal Capacitance (J/K):").grid(row=0, column=2, sticky='w', padx=5, pady=3)
        self.thermal_c_var = tk.DoubleVar(value=self.params.thermal_capacitance)
        ttk.Entry(params_frame, textvariable=self.thermal_c_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(params_frame, text="Max Temperature (°C):").grid(row=0, column=4, sticky='w', padx=5, pady=3)
        self.max_temp_var = tk.DoubleVar(value=self.params.max_temp)
        ttk.Entry(params_frame, textvariable=self.max_temp_var, width=10).grid(row=0, column=5, padx=5)

        # Mechanical parameters
        ttk.Label(params_frame, text="Moment of Inertia (kg·m²):").grid(row=1, column=0, sticky='w', padx=5, pady=3)
        self.inertia_var = tk.DoubleVar(value=self.params.moment_of_inertia)
        ttk.Entry(params_frame, textvariable=self.inertia_var, width=10).grid(row=1, column=1, padx=5)

        ttk.Label(params_frame, text="Friction Coefficient:").grid(row=1, column=2, sticky='w', padx=5, pady=3)
        self.friction_var = tk.DoubleVar(value=self.params.friction_coefficient)
        ttk.Entry(params_frame, textvariable=self.friction_var, width=10).grid(row=1, column=3, padx=5)

        ttk.Label(params_frame, text="Shaft Stiffness (N·m/rad):").grid(row=1, column=4, sticky='w', padx=5, pady=3)
        self.stiffness_var = tk.DoubleVar(value=self.params.shaft_stiffness)
        ttk.Entry(params_frame, textvariable=self.stiffness_var, width=10).grid(row=1, column=5, padx=5)

        ttk.Button(params_frame, text="Update Parameters",
                  command=self.update_multiphysics_params).grid(row=2, column=0, columnspan=6, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Multi-Physics Analysis Results", padding=10)
        results_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        self.multiphysics_text = scrolledtext.ScrolledText(results_frame, width=80, height=25,
                                                          font=('Courier', 9))
        self.multiphysics_text.pack(fill='both', expand=True)

    def create_losses_analysis_tab(self):
        """Losses breakdown analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Losses Analysis")

        # Create figure for losses pie chart and breakdown
        self.losses_fig = Figure(figsize=(12, 6), dpi=100)
        self.losses_canvas = FigureCanvasTkAgg(self.losses_fig, master=tab)
        self.losses_canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)

        # Create subplots
        self.ax_losses_pie = self.losses_fig.add_subplot(1, 2, 1)
        self.ax_losses_bar = self.losses_fig.add_subplot(1, 2, 2)

        self.losses_fig.tight_layout()

        # Control frame
        control_frame = ttk.Frame(tab)
        control_frame.pack(side='bottom', fill='x', padx=5, pady=5)

        ttk.Button(control_frame, text="Update Losses Analysis",
                  command=self.update_losses_analysis).pack(pady=5)

    def create_economic_analysis_tab(self):
        """Economic analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        # Input frame
        input_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        input_frame.pack(side='top', fill='x', padx=5, pady=5)

        ttk.Label(input_frame, text="Energy Cost ($/kWh):").grid(row=0, column=0, sticky='w', padx=5, pady=3)
        self.energy_cost_var = tk.DoubleVar(value=self.params.energy_cost)
        ttk.Entry(input_frame, textvariable=self.energy_cost_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Maintenance Cost ($/hr):").grid(row=0, column=2, sticky='w', padx=5, pady=3)
        self.maint_cost_var = tk.DoubleVar(value=self.params.maintenance_cost_per_hour)
        ttk.Entry(input_frame, textvariable=self.maint_cost_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(input_frame, text="Initial Cost ($):").grid(row=0, column=4, sticky='w', padx=5, pady=3)
        self.initial_cost_var = tk.DoubleVar(value=self.params.initial_cost)
        ttk.Entry(input_frame, textvariable=self.initial_cost_var, width=10).grid(row=0, column=5, padx=5)

        ttk.Label(input_frame, text="Operating Power (kW):").grid(row=1, column=0, sticky='w', padx=5, pady=3)
        self.op_power_var = tk.DoubleVar(value=10.0)
        ttk.Entry(input_frame, textvariable=self.op_power_var, width=10).grid(row=1, column=1, padx=5)

        ttk.Label(input_frame, text="Hours per Year:").grid(row=1, column=2, sticky='w', padx=5, pady=3)
        self.hours_year_var = tk.DoubleVar(value=4000)
        ttk.Entry(input_frame, textvariable=self.hours_year_var, width=10).grid(row=1, column=3, padx=5)

        ttk.Label(input_frame, text="Lifetime (years):").grid(row=1, column=4, sticky='w', padx=5, pady=3)
        self.lifetime_var = tk.DoubleVar(value=20)
        ttk.Entry(input_frame, textvariable=self.lifetime_var, width=10).grid(row=1, column=5, padx=5)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.update_economic_analysis).grid(row=2, column=0, columnspan=6, pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(tab, text="Economic Analysis Results", padding=10)
        results_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        self.economic_text = scrolledtext.ScrolledText(results_frame, width=80, height=20,
                                                       font=('Courier', 10))
        self.economic_text.pack(fill='both', expand=True)

    def create_advanced_controls_tab(self):
        """Advanced controls and thermal derating tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Advanced Controls")

        # Control strategies frame
        control_frame = ttk.LabelFrame(tab, text="Control Strategies", padding=10)
        control_frame.pack(side='top', fill='x', padx=5, pady=5)

        ttk.Label(control_frame, text="Voltage Control:").grid(row=0, column=0, sticky='w', padx=5, pady=3)
        self.voltage_control_var = tk.StringVar(value="AVR")
        voltage_combo = ttk.Combobox(control_frame, textvariable=self.voltage_control_var,
                                    values=['AVR', 'Manual', 'Droop'], width=15, state='readonly')
        voltage_combo.grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Frequency Control:").grid(row=0, column=2, sticky='w', padx=5, pady=3)
        self.freq_control_var = tk.StringVar(value="Governor")
        freq_combo = ttk.Combobox(control_frame, textvariable=self.freq_control_var,
                                 values=['Governor', 'Isochronous', 'Droop'], width=15, state='readonly')
        freq_combo.grid(row=0, column=3, padx=5)

        # Thermal derating frame
        derating_frame = ttk.LabelFrame(tab, text="Thermal Derating Analysis", padding=10)
        derating_frame.pack(side='top', fill='x', padx=5, pady=5)

        ttk.Label(derating_frame, text="Current Temperature (°C):").grid(row=0, column=0, sticky='w', padx=5, pady=3)
        self.current_temp_var = tk.DoubleVar(value=25.0)
        ttk.Entry(derating_frame, textvariable=self.current_temp_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(derating_frame, text="Derating Factor:").grid(row=0, column=2, sticky='w', padx=5, pady=3)
        self.derating_factor_label = ttk.Label(derating_frame, text="1.00", font=('Arial', 12, 'bold'))
        self.derating_factor_label.grid(row=0, column=3, padx=5)

        ttk.Button(derating_frame, text="Calculate Derating",
                  command=self.calculate_derating).grid(row=1, column=0, columnspan=4, pady=10)

        # Power consumption analysis frame
        power_frame = ttk.LabelFrame(tab, text="Power Consumption Analysis", padding=10)
        power_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        # Create figure for power consumption
        self.power_fig = Figure(figsize=(10, 5), dpi=100)
        self.power_canvas = FigureCanvasTkAgg(self.power_fig, master=power_frame)
        self.power_canvas.get_tk_widget().pack(fill='both', expand=True)

        self.ax_power_cons = self.power_fig.add_subplot(1, 1, 1)
        self.power_fig.tight_layout()

    def update_calculations(self):
        """Update all calculations and display results"""
        # Update calculator with new parameters
        self.calculator = AlternatorCalculator(self.params)

        # Clear results
        self.results_text.delete(1.0, tk.END)

        # Header
        self.results_text.insert(tk.END, "="*70 + "\n")
        self.results_text.insert(tk.END, "ALTERNATOR EMF CALCULATION RESULTS\n")
        self.results_text.insert(tk.END, "="*70 + "\n\n")

        # Machine configuration
        self.results_text.insert(tk.END, "MACHINE CONFIGURATION:\n")
        self.results_text.insert(tk.END, "-"*70 + "\n")
        self.results_text.insert(tk.END, f"Number of Poles:              {self.params.poles}\n")
        self.results_text.insert(tk.END, f"Frequency:                    {self.params.frequency} Hz\n")
        self.results_text.insert(tk.END, f"Speed:                        {self.params.speed_rpm} RPM\n")
        self.results_text.insert(tk.END, f"Total Slots:                  {self.calculator.total_slots}\n")
        self.results_text.insert(tk.END, f"Conductors per Slot:          {self.params.conductors_per_slot}\n")
        self.results_text.insert(tk.END, f"Total Conductors:             {self.calculator.total_conductors}\n")
        self.results_text.insert(tk.END, f"Conductors per Phase:         {self.calculator.conductors_per_phase}\n")
        self.results_text.insert(tk.END, f"Turns per Phase:              {self.calculator.turns_per_phase}\n")
        self.results_text.insert(tk.END, f"Winding Factor (Kw):          {self.params.winding_factor}\n")
        self.results_text.insert(tk.END, f"Flux per Pole:                {self.params.flux_per_pole} Wb\n\n")

        # AC Alternator calculations
        self.results_text.insert(tk.END, "AC ALTERNATOR (3-PHASE):\n")
        self.results_text.insert(tk.END, "-"*70 + "\n")

        # Star connection
        eph_star, vl_star = self.calculator.calculate_star_emf()
        self.results_text.insert(tk.END, "Star Connection:\n")
        self.results_text.insert(tk.END, f"  Phase Voltage (RMS):        {eph_star:.2f} V\n")
        self.results_text.insert(tk.END, f"  Line Voltage (RMS):         {vl_star:.2f} V\n")

        # Check if this matches the given terminal voltage
        if abs(vl_star - 1825) < 1:
            self.results_text.insert(tk.END, f"  ✓ Matches given terminal voltage (1825 V)\n")

        self.results_text.insert(tk.END, "\n")

        # Delta connection
        eph_delta, vl_delta = self.calculator.calculate_delta_emf()
        self.results_text.insert(tk.END, "Delta Connection:\n")
        self.results_text.insert(tk.END, f"  Phase Voltage (RMS):        {eph_delta:.2f} V\n")
        self.results_text.insert(tk.END, f"  Line Voltage (RMS):         {vl_delta:.2f} V\n\n")

        # DC Machine calculations
        self.results_text.insert(tk.END, "DC MACHINE EQUIVALENT:\n")
        self.results_text.insert(tk.END, "-"*70 + "\n")

        # Lap winding
        emf_lap = self.calculator.calculate_lap_winding_emf()
        self.results_text.insert(tk.END, "Lap Winding (A = P):\n")
        self.results_text.insert(tk.END, f"  Number of Parallel Paths:   {self.params.poles}\n")
        self.results_text.insert(tk.END, f"  EMF between Brushes:        {emf_lap:.2f} V\n\n")

        # Wave winding
        emf_wave = self.calculator.calculate_wave_winding_emf()
        self.results_text.insert(tk.END, "Wave Winding (A = 2):\n")
        self.results_text.insert(tk.END, f"  Number of Parallel Paths:   2\n")
        self.results_text.insert(tk.END, f"  EMF between Brushes:        {emf_wave:.2f} V\n\n")

        # Answer to the problem
        self.results_text.insert(tk.END, "="*70 + "\n")
        self.results_text.insert(tk.END, "ANSWER TO THE PROBLEM:\n")
        self.results_text.insert(tk.END, "="*70 + "\n")
        self.results_text.insert(tk.END, f"For lap-connected winding (DC machine configuration):\n")
        self.results_text.insert(tk.END, f"EMF between brushes = {emf_lap:.2f} V\n")
        self.results_text.insert(tk.END, f"\nAt the same speed ({self.params.speed_rpm} RPM) and same flux per pole\n")
        self.results_text.insert(tk.END, f"({self.params.flux_per_pole} Wb), the EMF is {emf_lap:.2f} V\n")
        self.results_text.insert(tk.END, "="*70 + "\n\n")

        # Performance metrics
        current_rms = 10.0  # Assume 10A for calculations
        losses = self.calculator.calculate_losses(current_rms, self.params.speed_rpm)

        self.results_text.insert(tk.END, "PERFORMANCE METRICS (at 10A load):\n")
        self.results_text.insert(tk.END, "-"*70 + "\n")
        self.results_text.insert(tk.END, f"Copper Losses:                {losses['copper']:.2f} W\n")
        self.results_text.insert(tk.END, f"Iron Losses:                  {losses['iron']:.2f} W\n")
        self.results_text.insert(tk.END, f"Mechanical Losses:            {losses['mechanical']:.2f} W\n")
        self.results_text.insert(tk.END, f"Stray Losses:                 {losses['stray']:.2f} W\n")
        self.results_text.insert(tk.END, f"Total Losses:                 {losses['total']:.2f} W\n")

        output_power = 3 * eph_star * current_rms * 0.8
        efficiency = self.calculator.calculate_efficiency(output_power, losses)
        self.results_text.insert(tk.END, f"\nOutput Power:                 {output_power:.2f} W\n")
        self.results_text.insert(tk.END, f"Efficiency:                   {efficiency:.2f} %\n")

    def start_simulation(self):
        """Start dynamic simulation"""
        if self.simulation_running:
            return

        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')

        # Get simulation parameters
        duration = self.sim_duration_var.get()
        dt = self.sim_dt_var.get()
        method = self.sim_method_var.get()

        # Reset progress
        self.sim_progress['value'] = 0

        # Run simulation in separate thread
        self.simulation_thread = threading.Thread(
            target=self._run_simulation,
            args=(duration, dt, method)
        )
        self.simulation_thread.start()

    def _run_simulation(self, duration, dt, method):
        """Run simulation in background thread"""
        def progress_callback(current, total):
            progress = (current / total) * 100
            self.sim_progress['value'] = progress
            self.root.update_idletasks()

        try:
            # Run simulation
            results = self.calculator.simulate_dynamic(duration, dt, method, progress_callback)

            # Update plots
            self.root.after(0, self._update_simulation_plots, results)

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {str(e)}")
        finally:
            self.simulation_running = False
            self.start_btn.config(state='normal')
            self.stop_btn.config(state='disabled')

    def _update_simulation_plots(self, results):
        """Update simulation plots with results"""
        t = results['time']

        # Clear all axes
        for ax in [self.ax_voltage, self.ax_current, self.ax_power,
                   self.ax_speed, self.ax_temp, self.ax_torque]:
            ax.clear()

        # Voltage plot
        self.ax_voltage.plot(t, results['voltage_rms'], 'b-', linewidth=2)
        self.ax_voltage.set_xlabel('Time (s)')
        self.ax_voltage.set_ylabel('Voltage (V RMS)')
        self.ax_voltage.set_title('Terminal Voltage')
        self.ax_voltage.grid(True, alpha=0.3)

        # Current plot
        self.ax_current.plot(t, results['current_a'], 'r-', label='Phase A', alpha=0.7)
        self.ax_current.plot(t, results['current_b'], 'g-', label='Phase B', alpha=0.7)
        self.ax_current.plot(t, results['current_c'], 'b-', label='Phase C', alpha=0.7)
        self.ax_current.plot(t, results['current_rms'], 'k-', label='RMS', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Phase Currents')
        self.ax_current.legend(loc='best', fontsize=8)
        self.ax_current.grid(True, alpha=0.3)

        # Power plot
        self.ax_power.plot(t, results['power'] / 1000, 'g-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.set_title('Output Power')
        self.ax_power.grid(True, alpha=0.3)

        # Speed plot
        self.ax_speed.plot(t, results['speed_rpm'], 'm-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.set_title('Rotor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        # Temperature plot
        self.ax_temp.plot(t, results['temperature'], 'r-', linewidth=2)
        self.ax_temp.axhline(y=self.params.max_temp, color='r', linestyle='--',
                            label=f'Max Temp ({self.params.max_temp}°C)')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Winding Temperature')
        self.ax_temp.legend(loc='best', fontsize=8)
        self.ax_temp.grid(True, alpha=0.3)

        # Torque plot
        self.ax_torque.plot(t, results['torque'], 'c-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.calculator.reset_simulation()
        self.sim_progress['value'] = 0

        # Clear plots
        for ax in [self.ax_voltage, self.ax_current, self.ax_power,
                   self.ax_speed, self.ax_temp, self.ax_torque]:
            ax.clear()
        self.sim_canvas.draw()

    def update_multiphysics_params(self):
        """Update multi-physics parameters"""
        self.params.thermal_resistance = self.thermal_r_var.get()
        self.params.thermal_capacitance = self.thermal_c_var.get()
        self.params.max_temp = self.max_temp_var.get()
        self.params.moment_of_inertia = self.inertia_var.get()
        self.params.friction_coefficient = self.friction_var.get()
        self.params.shaft_stiffness = self.stiffness_var.get()

        # Update display
        self.multiphysics_text.delete(1.0, tk.END)
        self.multiphysics_text.insert(tk.END, "="*70 + "\n")
        self.multiphysics_text.insert(tk.END, "MULTI-PHYSICS SIMULATION PARAMETERS\n")
        self.multiphysics_text.insert(tk.END, "="*70 + "\n\n")

        self.multiphysics_text.insert(tk.END, "ELECTROMAGNETIC MODEL:\n")
        self.multiphysics_text.insert(tk.END, "-"*70 + "\n")
        self.multiphysics_text.insert(tk.END, f"Resistance per Phase:         {self.params.resistance_per_phase} Ω\n")
        self.multiphysics_text.insert(tk.END, f"Inductance per Phase:         {self.params.inductance_per_phase} H\n")
        self.multiphysics_text.insert(tk.END, f"Back EMF Coefficient:         {4.44 * self.params.frequency * self.params.flux_per_pole * self.calculator.turns_per_phase:.2f} V/(rad/s)\n\n")

        self.multiphysics_text.insert(tk.END, "THERMAL MODEL:\n")
        self.multiphysics_text.insert(tk.END, "-"*70 + "\n")
        self.multiphysics_text.insert(tk.END, f"Thermal Resistance (Rth):     {self.params.thermal_resistance} K/W\n")
        self.multiphysics_text.insert(tk.END, f"Thermal Capacitance (Cth):    {self.params.thermal_capacitance} J/K\n")
        self.multiphysics_text.insert(tk.END, f"Thermal Time Constant:        {self.params.thermal_resistance * self.params.thermal_capacitance:.1f} s\n")
        self.multiphysics_text.insert(tk.END, f"Maximum Temperature:          {self.params.max_temp} °C\n")
        self.multiphysics_text.insert(tk.END, f"Ambient Temperature:          {self.params.ambient_temp} °C\n\n")

        self.multiphysics_text.insert(tk.END, "MECHANICAL MODEL:\n")
        self.multiphysics_text.insert(tk.END, "-"*70 + "\n")
        self.multiphysics_text.insert(tk.END, f"Moment of Inertia (J):        {self.params.moment_of_inertia} kg·m²\n")
        self.multiphysics_text.insert(tk.END, f"Friction Coefficient (B):     {self.params.friction_coefficient} N·m·s\n")
        self.multiphysics_text.insert(tk.END, f"Shaft Stiffness (K):          {self.params.shaft_stiffness:.2e} N·m/rad\n")
        self.multiphysics_text.insert(tk.END, f"Mechanical Time Constant:     {self.params.moment_of_inertia / self.params.friction_coefficient:.2f} s\n\n")

        self.multiphysics_text.insert(tk.END, "COUPLED DIFFERENTIAL EQUATIONS:\n")
        self.multiphysics_text.insert(tk.END, "-"*70 + "\n")
        self.multiphysics_text.insert(tk.END, "Electrical (per phase):\n")
        self.multiphysics_text.insert(tk.END, "  L·di/dt = E(ω,θ) - R(T)·i - V_load\n\n")
        self.multiphysics_text.insert(tk.END, "Mechanical:\n")
        self.multiphysics_text.insert(tk.END, "  J·dω/dt = Te(i,θ) - Tload - B·ω\n")
        self.multiphysics_text.insert(tk.END, "  dθ/dt = ω\n\n")
        self.multiphysics_text.insert(tk.END, "Thermal:\n")
        self.multiphysics_text.insert(tk.END, "  Cth·dT/dt = Ploss(i,T) - (T-Tamb)/Rth\n\n")

        self.multiphysics_text.insert(tk.END, "COUPLING EFFECTS:\n")
        self.multiphysics_text.insert(tk.END, "-"*70 + "\n")
        self.multiphysics_text.insert(tk.END, "• Resistance R varies with temperature T\n")
        self.multiphysics_text.insert(tk.END, "• Back EMF E depends on speed ω and position θ\n")
        self.multiphysics_text.insert(tk.END, "• Torque Te depends on current i and position θ\n")
        self.multiphysics_text.insert(tk.END, "• Losses Ploss depend on current i and temperature T\n")
        self.multiphysics_text.insert(tk.END, "• All variables interact in real-time through ODEs\n")

        messagebox.showinfo("Success", "Multi-physics parameters updated successfully!")

    def update_losses_analysis(self):
        """Update losses analysis"""
        current_rms = 10.0  # Example current
        losses = self.calculator.calculate_losses(current_rms, self.params.speed_rpm)

        # Clear axes
        self.ax_losses_pie.clear()
        self.ax_losses_bar.clear()

        # Pie chart
        labels = ['Copper\nLosses', 'Iron\nLosses', 'Mechanical\nLosses', 'Stray\nLosses']
        sizes = [losses['copper'], losses['iron'], losses['mechanical'], losses['stray']]
        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
        explode = (0.1, 0, 0, 0)

        self.ax_losses_pie.pie(sizes, explode=explode, labels=labels, colors=colors,
                              autopct='%1.1f%%', shadow=True, startangle=90)
        self.ax_losses_pie.set_title('Losses Distribution')

        # Bar chart
        loss_types = ['Copper', 'Iron', 'Mechanical', 'Stray', 'Total']
        loss_values = [losses['copper'], losses['iron'], losses['mechanical'],
                      losses['stray'], losses['total']]

        bars = self.ax_losses_bar.bar(loss_types, loss_values, color=colors + ['#ff6666'])
        self.ax_losses_bar.set_ylabel('Losses (W)')
        self.ax_losses_bar.set_title('Detailed Losses Breakdown')
        self.ax_losses_bar.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            self.ax_losses_bar.text(bar.get_x() + bar.get_width()/2., height,
                                   f'{height:.1f}W',
                                   ha='center', va='bottom')

        self.losses_fig.tight_layout()
        self.losses_canvas.draw()

    def update_economic_analysis(self):
        """Update economic analysis"""
        # Update parameters
        self.params.energy_cost = self.energy_cost_var.get()
        self.params.maintenance_cost_per_hour = self.maint_cost_var.get()
        self.params.initial_cost = self.initial_cost_var.get()

        # Get operating parameters
        power_kw = self.op_power_var.get()
        hours_year = self.hours_year_var.get()
        lifetime = self.lifetime_var.get()

        # Calculate costs
        annual_cost = self.economic_analyzer.calculate_operating_cost(power_kw, hours_year)
        lifecycle_cost = self.economic_analyzer.calculate_lifecycle_cost(lifetime, power_kw, hours_year)

        # Display results
        self.economic_text.delete(1.0, tk.END)
        self.economic_text.insert(tk.END, "="*70 + "\n")
        self.economic_text.insert(tk.END, "ECONOMIC ANALYSIS RESULTS\n")
        self.economic_text.insert(tk.END, "="*70 + "\n\n")

        self.economic_text.insert(tk.END, "INPUT PARAMETERS:\n")
        self.economic_text.insert(tk.END, "-"*70 + "\n")
        self.economic_text.insert(tk.END, f"Operating Power:              {power_kw} kW\n")
        self.economic_text.insert(tk.END, f"Operating Hours per Year:     {hours_year} hours\n")
        self.economic_text.insert(tk.END, f"Equipment Lifetime:           {lifetime} years\n")
        self.economic_text.insert(tk.END, f"Energy Cost:                  ${self.params.energy_cost}/kWh\n")
        self.economic_text.insert(tk.END, f"Maintenance Cost:             ${self.params.maintenance_cost_per_hour}/hour\n")
        self.economic_text.insert(tk.END, f"Initial Investment:           ${self.params.initial_cost:,.2f}\n\n")

        self.economic_text.insert(tk.END, "ANNUAL COSTS:\n")
        self.economic_text.insert(tk.END, "-"*70 + "\n")
        self.economic_text.insert(tk.END, f"Energy Cost:                  ${annual_cost['energy_cost']:,.2f}/year\n")
        self.economic_text.insert(tk.END, f"Maintenance Cost:             ${annual_cost['maintenance_cost']:,.2f}/year\n")
        self.economic_text.insert(tk.END, f"Total Annual Cost:            ${annual_cost['total_cost']:,.2f}/year\n")
        self.economic_text.insert(tk.END, f"Cost per kWh Generated:       ${annual_cost['cost_per_kwh']:.4f}/kWh\n\n")

        self.economic_text.insert(tk.END, "LIFECYCLE COSTS:\n")
        self.economic_text.insert(tk.END, "-"*70 + "\n")
        self.economic_text.insert(tk.END, f"Initial Cost:                 ${lifecycle_cost['initial_cost']:,.2f}\n")
        self.economic_text.insert(tk.END, f"Total Operating Cost:         ${lifecycle_cost['total_operating_cost']:,.2f}\n")
        self.economic_text.insert(tk.END, f"Total Lifecycle Cost:         ${lifecycle_cost['total_lifecycle_cost']:,.2f}\n")
        self.economic_text.insert(tk.END, f"Levelized Cost of Energy:     ${lifecycle_cost['levelized_cost_per_kwh']:.4f}/kWh\n\n")

        # Additional metrics
        total_energy = power_kw * hours_year * lifetime
        self.economic_text.insert(tk.END, "ADDITIONAL METRICS:\n")
        self.economic_text.insert(tk.END, "-"*70 + "\n")
        self.economic_text.insert(tk.END, f"Total Energy Generated:       {total_energy:,.0f} kWh\n")
        self.economic_text.insert(tk.END, f"Avg Cost per kWh (lifetime):  ${lifecycle_cost['total_lifecycle_cost']/total_energy:.4f}/kWh\n")

        # Sensitivity analysis
        self.economic_text.insert(tk.END, "\nSENSITIVITY ANALYSIS:\n")
        self.economic_text.insert(tk.END, "-"*70 + "\n")

        # Energy cost sensitivity
        for factor in [0.8, 0.9, 1.1, 1.2]:
            temp_params = AlternatorParameters()
            temp_params.energy_cost = self.params.energy_cost * factor
            temp_params.maintenance_cost_per_hour = self.params.maintenance_cost_per_hour
            temp_params.initial_cost = self.params.initial_cost
            temp_analyzer = EconomicAnalyzer(temp_params)
            temp_lifecycle = temp_analyzer.calculate_lifecycle_cost(lifetime, power_kw, hours_year)
            change = (factor - 1) * 100
            self.economic_text.insert(tk.END,
                f"Energy cost {change:+.0f}%:            ${temp_lifecycle['levelized_cost_per_kwh']:.4f}/kWh\n")

    def calculate_derating(self):
        """Calculate thermal derating factor"""
        current_temp = self.current_temp_var.get()
        max_temp = self.params.max_temp
        rated_temp = 40.0  # Rated ambient temperature

        if current_temp <= rated_temp:
            derating_factor = 1.0
        elif current_temp >= max_temp:
            derating_factor = 0.0
        else:
            # Linear derating
            derating_factor = 1.0 - (current_temp - rated_temp) / (max_temp - rated_temp)

        self.derating_factor_label.config(text=f"{derating_factor:.3f}")

        # Update power consumption plot
        self.ax_power_cons.clear()

        temps = np.linspace(0, max_temp + 20, 100)
        derating = np.zeros_like(temps)

        for i, t in enumerate(temps):
            if t <= rated_temp:
                derating[i] = 1.0
            elif t >= max_temp:
                derating[i] = 0.0
            else:
                derating[i] = 1.0 - (t - rated_temp) / (max_temp - rated_temp)

        self.ax_power_cons.plot(temps, derating * 100, 'b-', linewidth=2)
        self.ax_power_cons.axvline(x=current_temp, color='r', linestyle='--',
                                   label=f'Current: {current_temp}°C')
        self.ax_power_cons.axvline(x=rated_temp, color='g', linestyle='--',
                                   label=f'Rated: {rated_temp}°C')
        self.ax_power_cons.axvline(x=max_temp, color='r', linestyle='-',
                                   label=f'Maximum: {max_temp}°C')
        self.ax_power_cons.fill_between(temps, 0, derating * 100,
                                        where=(temps >= rated_temp) & (temps <= max_temp),
                                        alpha=0.3, color='yellow', label='Derating Zone')

        self.ax_power_cons.set_xlabel('Temperature (°C)')
        self.ax_power_cons.set_ylabel('Power Capacity (%)')
        self.ax_power_cons.set_title('Thermal Derating Curve')
        self.ax_power_cons.legend(loc='best')
        self.ax_power_cons.grid(True, alpha=0.3)
        self.ax_power_cons.set_ylim([0, 105])

        self.power_fig.tight_layout()
        self.power_canvas.draw()

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # This is called on every configure event
        # We only update if it's the root window
        if event.widget == self.root:
            # The plots will auto-scale due to pack with fill='both' and expand=True
            pass

    def save_configuration(self):
        """Save current configuration to JSON file"""
        config = {
            'poles': self.params.poles,
            'frequency': self.params.frequency,
            'slots_per_pole': self.params.slots_per_pole,
            'conductors_per_slot': self.params.conductors_per_slot,
            'winding_factor': self.params.winding_factor,
            'flux_per_pole': self.params.flux_per_pole,
            'speed_rpm': self.params.speed_rpm,
            'resistance_per_phase': self.params.resistance_per_phase,
            'inductance_per_phase': self.params.inductance_per_phase,
            'load_resistance': self.params.load_resistance,
            'load_inductance': self.params.load_inductance,
        }

        try:
            with open('alternator_config.json', 'w') as f:
                json.dump(config, f, indent=4)
            messagebox.showinfo("Success", "Configuration saved to alternator_config.json")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save configuration: {str(e)}")

    def load_configuration(self):
        """Load configuration from JSON file"""
        try:
            with open('alternator_config.json', 'r') as f:
                config = json.load(f)

            # Update parameters
            for key, value in config.items():
                if hasattr(self.params, key):
                    setattr(self.params, key, value)

            # Update GUI
            self.update_calculations()
            messagebox.showinfo("Success", "Configuration loaded from alternator_config.json")
        except FileNotFoundError:
            messagebox.showerror("Error", "Configuration file not found")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load configuration: {str(e)}")

    def show_about(self):
        """Show about dialog"""
        about_text = """
        Advanced Alternator & Electrical Machine Simulator
        Version 1.0

        Multi-Physics Simulation Platform

        Features:
        • AC/DC machine EMF calculations
        • Dynamic simulation with multiple ODE solvers (RK45, Euler)
        • Coupled electromagnetic-thermal-mechanical analysis
        • Detailed losses breakdown
        • Economic analysis
        • Advanced control strategies
        • Thermal derating analysis

        Developed for electrical engineering education and research.
        """
        messagebox.showinfo("About", about_text)

    def show_documentation(self):
        """Show documentation"""
        doc_text = """
        DOCUMENTATION

        1. MAIN CALCULATIONS
           - Configure alternator parameters using sliders
           - View EMF calculations for different winding configurations

        2. DYNAMIC SIMULATION
           - Select ODE solver (RK45 recommended for accuracy, Euler for speed)
           - Set simulation duration and time step
           - View real-time plots of voltage, current, power, etc.

        3. MULTI-PHYSICS ANALYSIS
           - Coupled electromagnetic, thermal, and mechanical models
           - Temperature-dependent resistance
           - Real-time thermal dynamics

        4. LOSSES ANALYSIS
           - Detailed breakdown of copper, iron, mechanical, and stray losses
           - Visual representation with pie and bar charts

        5. ECONOMIC ANALYSIS
           - Operating and lifecycle costs
           - Sensitivity analysis
           - Levelized cost of energy

        6. ADVANCED CONTROLS
           - Voltage and frequency control strategies
           - Thermal derating curves
           - Power consumption analysis
        """

        # Create new window for documentation
        doc_window = tk.Toplevel(self.root)
        doc_window.title("Documentation")
        doc_window.geometry("600x500")

        text = scrolledtext.ScrolledText(doc_window, wrap=tk.WORD, font=('Arial', 10))
        text.pack(fill='both', expand=True, padx=10, pady=10)
        text.insert(tk.END, doc_text)
        text.config(state='disabled')


def main():
    """Main entry point"""
    root = tk.Tk()

    # Configure style
    style = ttk.Style()
    style.theme_use('clam')

    # Create application
    app = AlternatorSimulatorGUI(root)

    # Run
    root.mainloop()


if __name__ == "__main__":
    main()
