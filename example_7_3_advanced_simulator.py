"""
Advanced Salient-Pole Synchronous Generator Simulator
Example 7.3 Solution with Multi-Physics Interactive Lab

Features:
- Theoretical calculations for Example 7.3
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Dynamic ODE solvers (RK45, Euler)
- Real-time interactive controls
- Economic analysis and loss breakdown
- Auto-scaling visualization
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import ipywidgets as widgets
from ipywidgets import Layout, VBox, HBox, Tab, Button, FloatSlider, Dropdown, Output, HTML
from IPython.display import display, clear_output
from scipy.integrate import solve_ivp, odeint
from scipy.interpolate import interp1d
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# EXAMPLE 7.3 THEORETICAL SOLUTION
# =============================================================================

class Example73Solution:
    """Solve Example 7.3 - Salient-Pole Synchronous Generator Analysis"""

    def __init__(self):
        # Nominal parameters
        self.Sn = 50e3  # VA
        self.V1Ln = 380  # V (line voltage)
        self.fn = 60  # Hz
        self.nn = 1800  # rpm
        self.cos_phi_n = 0.82

        # Test data
        self.n_slip = 1768  # rpm (slip test speed)
        self.Vs = 115  # V (slip test voltage)
        self.fs = 60  # Hz
        self.Imax = 22.2  # A
        self.Imin = 11.3  # A
        self.Rf_20C = 0.8  # Ω
        self.If0 = 10.5  # A (field current for nominal voltage at no-load)

        # Constants
        self.alpha_copper = 0.00393  # Temperature coefficient for copper (1/°C)
        self.T_ref = 20  # °C

        # Calculate derived parameters
        self.calculate_all()

    def calculate_all(self):
        """Perform all calculations"""
        # Phase voltage
        self.Vph = self.V1Ln / np.sqrt(3)  # Phase voltage

        # Nominal current
        self.In = self.Sn / (np.sqrt(3) * self.V1Ln)

        # Power factor angle
        self.phi_n = np.arccos(self.cos_phi_n)
        self.sin_phi_n = np.sin(self.phi_n)

        # (a) Calculate synchronous reactances using slip test
        self.Xsd = self.Vs / self.Imin  # Direct-axis synchronous reactance
        self.Xsq = self.Vs / self.Imax  # Quadrature-axis synchronous reactance

        # Average reactance
        self.Xs_avg = (self.Xsd + self.Xsq) / 2

        # (b) Calculate nominal field excitation current
        # Decompose current into d-q components
        self.Id = self.In * np.sin(self.phi_n)  # Direct-axis current
        self.Iq = self.In * np.cos(self.phi_n)  # Quadrature-axis current

        # Excitation EMF (neglecting stator resistance)
        self.Ed = self.Vph * np.cos(self.phi_n) + self.Xsq * self.Iq
        self.Eq = self.Vph * np.sin(self.phi_n) - self.Xsd * self.Id

        # Magnitude of excitation EMF
        self.E0 = np.sqrt(self.Ed**2 + self.Eq**2)

        # At no-load, E0 = Vph when If = If0
        # Therefore, the field excitation is proportional to EMF
        self.Ifn = self.If0 * (self.E0 / self.Vph)

        # (c) Calculate field voltage at operating temperature
        self.calculate_field_voltage(120)  # At 120°C

    def calculate_field_voltage(self, temperature):
        """Calculate field winding voltage at given temperature"""
        # Correct resistance for temperature
        self.Rf_t = self.Rf_20C * (1 + self.alpha_copper * (temperature - self.T_ref))

        # Field winding voltage
        self.Vf_t = self.Ifn * self.Rf_t

        return self.Vf_t

    def get_results(self):
        """Return formatted results"""
        results = {
            'Xsd': self.Xsd,
            'Xsq': self.Xsq,
            'Ifn': self.Ifn,
            'Vf_120C': self.Vf_t,
            'Rf_120C': self.Rf_t,
            'E0': self.E0,
            'Vph': self.Vph,
            'In': self.In,
            'Id': self.Id,
            'Iq': self.Iq,
            'phi_n': np.degrees(self.phi_n)
        }
        return results

    def print_results(self):
        """Print formatted results"""
        results = self.get_results()

        print("=" * 80)
        print("EXAMPLE 7.3 - SALIENT-POLE SYNCHRONOUS GENERATOR SOLUTION")
        print("=" * 80)
        print("\n(a) SYNCHRONOUS REACTANCES:")
        print(f"    Xsd (Direct-axis reactance)     = {results['Xsd']:.4f} Ω")
        print(f"    Xsq (Quadrature-axis reactance) = {results['Xsq']:.4f} Ω")

        print("\n(b) NOMINAL FIELD EXCITATION CURRENT:")
        print(f"    Ifn = {results['Ifn']:.4f} A")

        print("\n(c) FIELD WINDING VOLTAGE AT 120°C:")
        print(f"    Rf at 120°C = {results['Rf_120C']:.4f} Ω")
        print(f"    Vf at 120°C = {results['Vf_120C']:.4f} V")

        print("\nADDITIONAL RESULTS:")
        print(f"    Phase voltage (Vph)        = {results['Vph']:.2f} V")
        print(f"    Nominal current (In)       = {results['In']:.2f} A")
        print(f"    Excitation EMF (E0)        = {results['E0']:.2f} V")
        print(f"    Direct-axis current (Id)   = {results['Id']:.2f} A")
        print(f"    Quadrature-axis current (Iq) = {results['Iq']:.2f} A")
        print(f"    Power factor angle         = {results['phi_n']:.2f}°")
        print("=" * 80)


# =============================================================================
# MULTI-PHYSICS SIMULATION ENGINE
# =============================================================================

class SynchronousGeneratorMultiPhysics:
    """
    Advanced Multi-Physics Simulation of Salient-Pole Synchronous Generator
    Includes: Electromagnetic, Thermal, and Mechanical models
    """

    def __init__(self, example_solution):
        self.ex = example_solution

        # Machine parameters
        self.p = int(60 * self.ex.fn / self.ex.nn)  # Number of pole pairs
        self.omega_s = 2 * np.pi * self.ex.fn  # Synchronous angular frequency
        self.omega_m_rated = 2 * np.pi * self.ex.nn / 60  # Mechanical angular velocity

        # Electrical parameters
        self.Xsd = self.ex.Xsd
        self.Xsq = self.ex.Xsq
        self.Ra = 0.05  # Stator resistance (estimated, small)
        self.Lsd = self.Xsd / self.omega_s
        self.Lsq = self.Xsq / self.omega_s
        self.Lm = 0.9 * self.Lsd  # Mutual inductance

        # Thermal parameters
        self.thermal_resistance_stator = 0.15  # K/W
        self.thermal_capacitance_stator = 5000  # J/K
        self.thermal_resistance_rotor = 0.20  # K/W
        self.thermal_capacitance_rotor = 3000  # J/K
        self.ambient_temp = 25  # °C

        # Mechanical parameters
        self.J = 0.5  # Moment of inertia (kg·m²)
        self.B = 0.01  # Damping coefficient (N·m·s)

        # Loss coefficients
        self.k_copper = 1.0
        self.k_iron = 0.02  # Iron loss coefficient
        self.k_friction = 0.005  # Friction coefficient
        self.k_stray = 0.01  # Stray loss coefficient

        # Operating conditions
        self.If = self.ex.Ifn
        self.omega_m = self.omega_m_rated
        self.load_current = self.ex.In
        self.power_factor = self.ex.cos_phi_n

        # State variables
        self.reset_state()

    def reset_state(self):
        """Reset all state variables"""
        self.time = 0
        self.T_stator = self.ambient_temp
        self.T_rotor = self.ambient_temp
        self.theta_m = 0  # Mechanical angle
        self.history = {
            'time': [],
            'id': [], 'iq': [], 'vd': [], 'vq': [],
            'Te': [], 'Tm': [], 'omega_m': [],
            'T_stator': [], 'T_rotor': [],
            'P_copper': [], 'P_iron': [], 'P_friction': [], 'P_stray': [],
            'P_out': [], 'efficiency': [],
            'If': [], 'E0': []
        }

    def calculate_dq_currents(self, load_power_factor=None):
        """Calculate d-q axis currents"""
        if load_power_factor is None:
            load_power_factor = self.power_factor

        phi = np.arccos(load_power_factor)
        Id = self.load_current * np.sin(phi)
        Iq = self.load_current * np.cos(phi)

        return Id, Iq

    def calculate_emf(self, If):
        """Calculate excitation EMF as function of field current"""
        # Linear magnetization curve
        return (self.ex.Vph / self.ex.If0) * If

    def calculate_electromagnetic_torque(self, Id, Iq, If):
        """Calculate electromagnetic torque"""
        E0 = self.calculate_emf(If)
        # Torque equation for salient-pole machine
        Te = (3 * self.p / 2) * (E0 * Iq / self.omega_s +
                                   (self.Lsd - self.Lsq) * Id * Iq)
        return Te

    def calculate_losses(self, Id, Iq, omega_m):
        """Calculate detailed loss breakdown"""
        # Copper losses
        I_stator_rms = np.sqrt(Id**2 + Iq**2)
        P_copper_stator = 3 * self.Ra * I_stator_rms**2 * self.k_copper
        P_copper_rotor = self.ex.Rf_t * self.If**2
        P_copper_total = P_copper_stator + P_copper_rotor

        # Iron losses (frequency and flux dependent)
        f_actual = omega_m * self.p / (2 * np.pi)
        P_iron = self.k_iron * (f_actual / self.ex.fn)**2 * self.ex.Sn

        # Mechanical friction losses
        P_friction = self.k_friction * omega_m**2

        # Stray load losses
        P_stray = self.k_stray * I_stator_rms**2

        return {
            'P_copper': P_copper_total,
            'P_copper_stator': P_copper_stator,
            'P_copper_rotor': P_copper_rotor,
            'P_iron': P_iron,
            'P_friction': P_friction,
            'P_stray': P_stray,
            'P_total': P_copper_total + P_iron + P_friction + P_stray
        }

    def thermal_model(self, T_stator, T_rotor, P_losses):
        """Thermal differential equations"""
        # Heat generation
        P_stator_heat = P_losses['P_copper_stator'] + P_losses['P_iron']
        P_rotor_heat = P_losses['P_copper_rotor']

        # Temperature derivatives
        dT_stator_dt = (P_stator_heat - (T_stator - self.ambient_temp) /
                        self.thermal_resistance_stator) / self.thermal_capacitance_stator

        dT_rotor_dt = (P_rotor_heat - (T_rotor - self.ambient_temp) /
                       self.thermal_resistance_rotor) / self.thermal_capacitance_rotor

        return dT_stator_dt, dT_rotor_dt

    def mechanical_model(self, Te, Tm, omega_m):
        """Mechanical differential equations"""
        # Equation: J * dω/dt = Te - Tm - B * ω
        domega_dt = (Te - Tm - self.B * omega_m) / self.J
        dtheta_dt = omega_m

        return domega_dt, dtheta_dt

    def system_ode(self, t, y, Tm_func, If_func, load_pf):
        """
        Complete system ODEs for multi-physics simulation
        State vector y = [theta_m, omega_m, T_stator, T_rotor]
        """
        theta_m, omega_m, T_stator, T_rotor = y

        # Get time-varying inputs
        Tm = Tm_func(t) if callable(Tm_func) else Tm_func
        If = If_func(t) if callable(If_func) else If_func

        # Update field resistance with temperature
        self.ex.calculate_field_voltage((T_rotor + T_stator) / 2)

        # Calculate currents
        Id, Iq = self.calculate_dq_currents(load_pf)

        # Electromagnetic torque
        Te = self.calculate_electromagnetic_torque(Id, Iq, If)

        # Mechanical dynamics
        domega_dt, dtheta_dt = self.mechanical_model(Te, Tm, omega_m)

        # Calculate losses
        P_losses = self.calculate_losses(Id, Iq, omega_m)

        # Thermal dynamics
        dT_stator_dt, dT_rotor_dt = self.thermal_model(T_stator, T_rotor, P_losses)

        return [dtheta_dt, domega_dt, dT_stator_dt, dT_rotor_dt]

    def simulate_dynamics(self, t_span, Tm_profile, If_profile, load_pf,
                         method='RK45', dt=0.01):
        """
        Run dynamic simulation with ODE solver

        Parameters:
        -----------
        t_span : tuple
            (t_start, t_end)
        Tm_profile : callable or float
            Mechanical torque as function of time
        If_profile : callable or float
            Field current as function of time
        load_pf : float
            Load power factor
        method : str
            'RK45' or 'Euler'
        dt : float
            Time step for Euler method
        """
        self.reset_state()

        # Initial conditions
        y0 = [self.theta_m, self.omega_m, self.T_stator, self.T_rotor]

        if method == 'RK45':
            # Use scipy's adaptive RK45 solver
            sol = solve_ivp(
                lambda t, y: self.system_ode(t, y, Tm_profile, If_profile, load_pf),
                t_span,
                y0,
                method='RK45',
                max_step=dt,
                dense_output=True
            )

            # Extract solution
            t_eval = np.arange(t_span[0], t_span[1], dt)
            y_sol = sol.sol(t_eval)

        elif method == 'Euler':
            # Forward Euler method
            t_eval = np.arange(t_span[0], t_span[1], dt)
            y_sol = np.zeros((4, len(t_eval)))
            y_sol[:, 0] = y0

            for i in range(1, len(t_eval)):
                t = t_eval[i-1]
                y = y_sol[:, i-1]
                dydt = self.system_ode(t, y, Tm_profile, If_profile, load_pf)
                y_sol[:, i] = y + dt * np.array(dydt)

        else:
            raise ValueError("Method must be 'RK45' or 'Euler'")

        # Store results
        for i, t in enumerate(t_eval):
            theta_m, omega_m, T_stator, T_rotor = y_sol[:, i]

            # Get currents and field
            If_t = If_profile(t) if callable(If_profile) else If_profile
            Id, Iq = self.calculate_dq_currents(load_pf)

            # Calculate quantities
            Te = self.calculate_electromagnetic_torque(Id, Iq, If_t)
            Tm = Tm_profile(t) if callable(Tm_profile) else Tm_profile
            E0 = self.calculate_emf(If_t)

            # Losses
            P_losses = self.calculate_losses(Id, Iq, omega_m)

            # Output power
            P_out = Te * omega_m
            P_in = P_out + P_losses['P_total']
            efficiency = (P_out / P_in * 100) if P_in > 0 else 0

            # Voltages
            Vd = self.ex.Vph * np.cos(np.arccos(load_pf))
            Vq = self.ex.Vph * np.sin(np.arccos(load_pf))

            # Store history
            self.history['time'].append(t)
            self.history['id'].append(Id)
            self.history['iq'].append(Iq)
            self.history['vd'].append(Vd)
            self.history['vq'].append(Vq)
            self.history['Te'].append(Te)
            self.history['Tm'].append(Tm)
            self.history['omega_m'].append(omega_m)
            self.history['T_stator'].append(T_stator)
            self.history['T_rotor'].append(T_rotor)
            self.history['P_copper'].append(P_losses['P_copper'])
            self.history['P_iron'].append(P_losses['P_iron'])
            self.history['P_friction'].append(P_losses['P_friction'])
            self.history['P_stray'].append(P_losses['P_stray'])
            self.history['P_out'].append(P_out)
            self.history['efficiency'].append(efficiency)
            self.history['If'].append(If_t)
            self.history['E0'].append(E0)

        return t_eval, y_sol


# =============================================================================
# ECONOMIC ANALYSIS MODULE
# =============================================================================

class EconomicAnalysis:
    """Economic analysis and cost calculations"""

    def __init__(self, generator_system):
        self.gen = generator_system

        # Economic parameters (USD)
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost_annual = 500  # $/year
        self.capital_cost = 5000  # $ (generator cost)
        self.lifetime_years = 20
        self.discount_rate = 0.05

    def calculate_energy_cost(self, operating_hours_per_year):
        """Calculate annual energy cost"""
        if len(self.gen.history['P_out']) == 0:
            return 0

        # Average power loss
        avg_P_loss = np.mean([
            self.gen.history['P_copper'][-1] +
            self.gen.history['P_iron'][-1] +
            self.gen.history['P_friction'][-1] +
            self.gen.history['P_stray'][-1]
        ])

        # Annual energy loss
        E_loss_annual = (avg_P_loss / 1000) * operating_hours_per_year  # kWh

        # Cost
        cost_annual = E_loss_annual * self.electricity_cost

        return cost_annual, E_loss_annual

    def calculate_lcoe(self, operating_hours_per_year):
        """Calculate Levelized Cost of Energy"""
        cost_annual, E_loss_annual = self.calculate_energy_cost(operating_hours_per_year)

        # Total annual cost
        total_annual_cost = cost_annual + self.maintenance_cost_annual

        # Present value of costs
        pv_costs = self.capital_cost
        for year in range(1, self.lifetime_years + 1):
            pv_costs += total_annual_cost / (1 + self.discount_rate)**year

        # Energy generated
        avg_P_out = np.mean(self.gen.history['P_out']) if len(self.gen.history['P_out']) > 0 else self.gen.ex.Sn
        E_annual = (avg_P_out / 1000) * operating_hours_per_year  # kWh

        # Present value of energy
        pv_energy = 0
        for year in range(1, self.lifetime_years + 1):
            pv_energy += E_annual / (1 + self.discount_rate)**year

        # LCOE
        lcoe = pv_costs / pv_energy if pv_energy > 0 else 0

        return lcoe, pv_costs, pv_energy

    def calculate_payback_period(self, revenue_per_kwh):
        """Calculate simple payback period"""
        avg_P_out = np.mean(self.gen.history['P_out']) if len(self.gen.history['P_out']) > 0 else self.gen.ex.Sn
        annual_revenue = (avg_P_out / 1000) * 8760 * revenue_per_kwh  # Assume full year operation

        annual_cost = self.maintenance_cost_annual
        net_annual = annual_revenue - annual_cost

        if net_annual > 0:
            payback = self.capital_cost / net_annual
        else:
            payback = float('inf')

        return payback


# =============================================================================
# INTERACTIVE SIMULATOR GUI
# =============================================================================

class AdvancedGeneratorSimulatorGUI:
    """
    Advanced Interactive Simulator with ipywidgets
    Features: Multi-physics simulation, economic analysis, dynamic visualization
    """

    def __init__(self):
        # Initialize solution and simulator
        self.solution = Example73Solution()
        self.generator = SynchronousGeneratorMultiPhysics(self.solution)
        self.economics = EconomicAnalysis(self.generator)

        # Simulation control
        self.is_running = False
        self.current_time = 0
        self.sim_method = 'RK45'

        # Create GUI
        self.create_gui()

    def create_gui(self):
        """Create the complete GUI"""

        # =====================================================================
        # STYLE AND LAYOUT
        # =====================================================================

        style = {'description_width': '180px'}
        slider_layout = Layout(width='400px')
        button_layout = Layout(width='100px', height='40px')

        # =====================================================================
        # HEADER
        # =====================================================================

        header = HTML(value="""
        <h1 style='text-align: center; color: #2E86AB; font-family: Arial;'>
            🔌 Advanced Salient-Pole Synchronous Generator Simulator
        </h1>
        <h3 style='text-align: center; color: #A23B72;'>
            Example 7.3 - Multi-Physics Interactive Lab
        </h3>
        <hr style='border: 2px solid #2E86AB;'>
        """)

        # =====================================================================
        # CONTROL PANEL - INPUT PARAMETERS
        # =====================================================================

        self.load_power_slider = FloatSlider(
            value=self.solution.cos_phi_n,
            min=0.5, max=1.0, step=0.01,
            description='Power Factor:',
            style=style, layout=slider_layout,
            readout_format='.2f'
        )

        self.load_current_slider = FloatSlider(
            value=self.solution.In,
            min=0, max=self.solution.In * 1.5, step=1,
            description='Load Current (A):',
            style=style, layout=slider_layout,
            readout_format='.1f'
        )

        self.field_current_slider = FloatSlider(
            value=self.solution.Ifn,
            min=0, max=20, step=0.1,
            description='Field Current (A):',
            style=style, layout=slider_layout,
            readout_format='.2f'
        )

        self.speed_slider = FloatSlider(
            value=self.solution.nn,
            min=0, max=2400, step=10,
            description='Speed (rpm):',
            style=style, layout=slider_layout,
            readout_format='.0f'
        )

        self.torque_slider = FloatSlider(
            value=200,
            min=0, max=500, step=10,
            description='Mech. Torque (Nm):',
            style=style, layout=slider_layout,
            readout_format='.0f'
        )

        self.ambient_temp_slider = FloatSlider(
            value=25,
            min=0, max=50, step=1,
            description='Ambient Temp (°C):',
            style=style, layout=slider_layout,
            readout_format='.0f'
        )

        self.sim_time_slider = FloatSlider(
            value=5.0,
            min=0.1, max=20.0, step=0.1,
            description='Simulation Time (s):',
            style=style, layout=slider_layout,
            readout_format='.1f'
        )

        self.solver_dropdown = Dropdown(
            options=['RK45', 'Euler'],
            value='RK45',
            description='ODE Solver:',
            style=style
        )

        # =====================================================================
        # CONTROL BUTTONS
        # =====================================================================

        self.start_button = Button(
            description='▶ START',
            button_style='success',
            layout=button_layout,
            icon='play'
        )

        self.stop_button = Button(
            description='⏸ STOP',
            button_style='warning',
            layout=button_layout,
            icon='pause',
            disabled=True
        )

        self.reset_button = Button(
            description='🔄 RESET',
            button_style='danger',
            layout=button_layout,
            icon='refresh'
        )

        self.calculate_button = Button(
            description='📊 CALCULATE',
            button_style='info',
            layout=Layout(width='150px', height='40px')
        )

        # Button callbacks
        self.start_button.on_click(self.on_start)
        self.stop_button.on_click(self.on_stop)
        self.reset_button.on_click(self.on_reset)
        self.calculate_button.on_click(self.on_calculate)

        # =====================================================================
        # OUTPUT DISPLAYS
        # =====================================================================

        self.output_display = Output(layout=Layout(
            border='2px solid #2E86AB',
            padding='10px',
            width='100%',
            height='auto'
        ))

        self.plot_output = Output(layout=Layout(
            width='100%',
            height='auto'
        ))

        self.economic_output = Output(layout=Layout(
            border='2px solid #A23B72',
            padding='10px',
            width='100%'
        ))

        self.loss_output = Output(layout=Layout(
            border='2px solid #F18F01',
            padding='10px',
            width='100%'
        ))

        # =====================================================================
        # ORGANIZE IN TABS
        # =====================================================================

        # Tab 1: Main Controls
        controls_box = VBox([
            HTML('<h3 style="color: #2E86AB;">⚙️ Control Parameters</h3>'),
            self.load_power_slider,
            self.load_current_slider,
            self.field_current_slider,
            self.speed_slider,
            self.torque_slider,
            self.ambient_temp_slider,
            self.sim_time_slider,
            self.solver_dropdown,
            HBox([self.start_button, self.stop_button, self.reset_button,
                  self.calculate_button]),
        ])

        # Tab 2: Results Display
        results_box = VBox([
            HTML('<h3 style="color: #2E86AB;">📈 Simulation Results</h3>'),
            self.output_display,
            self.plot_output
        ])

        # Tab 3: Economic Analysis
        economic_box = VBox([
            HTML('<h3 style="color: #A23B72;">💰 Economic Analysis</h3>'),
            HTML("""
            <p style='font-family: Arial; font-size: 14px;'>
            This section provides comprehensive economic analysis including:
            <ul>
                <li>Annual energy costs and losses</li>
                <li>Levelized Cost of Energy (LCOE)</li>
                <li>Payback period analysis</li>
                <li>Present value calculations</li>
            </ul>
            </p>
            """),
            self.economic_output
        ])

        # Tab 4: Loss Breakdown
        loss_box = VBox([
            HTML('<h3 style="color: #F18F01;">⚡ Loss Breakdown & Efficiency</h3>'),
            HTML("""
            <p style='font-family: Arial; font-size: 14px;'>
            Detailed analysis of all losses:
            <ul>
                <li>Copper losses (stator & rotor)</li>
                <li>Iron/core losses</li>
                <li>Mechanical friction losses</li>
                <li>Stray load losses</li>
            </ul>
            </p>
            """),
            self.loss_output
        ])

        # Tab 5: Theory
        theory_box = VBox([
            HTML("""
            <h3 style="color: #2E86AB;">📚 Example 7.3 - Theory</h3>
            <div style='font-family: Arial; font-size: 13px; padding: 10px;'>
                <h4>Problem Statement:</h4>
                <p>A salient-pole synchronous generator with nominal parameters:</p>
                <ul>
                    <li>Apparent power: 50 kVA</li>
                    <li>Line voltage: 380 V</li>
                    <li>Frequency: 60 Hz</li>
                    <li>Speed: 1800 rpm</li>
                    <li>Power factor: 0.82</li>
                </ul>

                <h4>Slip Test Results:</h4>
                <ul>
                    <li>Test voltage: 115 V at 60 Hz</li>
                    <li>Maximum current: 22.2 A (q-axis aligned)</li>
                    <li>Minimum current: 11.3 A (d-axis aligned)</li>
                </ul>

                <h4>Solution Method:</h4>
                <p><b>(a) Synchronous Reactances:</b></p>
                <p>X<sub>sd</sub> = V<sub>s</sub> / I<sub>min</sub> (direct-axis)</p>
                <p>X<sub>sq</sub> = V<sub>s</sub> / I<sub>max</sub> (quadrature-axis)</p>

                <p><b>(b) Field Excitation Current:</b></p>
                <p>Using d-q decomposition and voltage equations with linear magnetization</p>

                <p><b>(c) Field Voltage:</b></p>
                <p>R<sub>f</sub>(T) = R<sub>f</sub>(20°C) × [1 + α(T - 20°C)]</p>
                <p>V<sub>f</sub> = I<sub>fn</sub> × R<sub>f</sub>(T)</p>
            </div>
            """)
        ])

        # Create tabs
        self.tabs = Tab()
        self.tabs.children = [controls_box, results_box, economic_box, loss_box, theory_box]
        self.tabs.set_title(0, '⚙️ Controls')
        self.tabs.set_title(1, '📊 Results')
        self.tabs.set_title(2, '💰 Economics')
        self.tabs.set_title(3, '⚡ Losses')
        self.tabs.set_title(4, '📚 Theory')

        # =====================================================================
        # FINAL LAYOUT
        # =====================================================================

        self.main_widget = VBox([
            header,
            self.tabs
        ])

        # Display initial results
        self.display_theoretical_solution()

    def update_generator_parameters(self):
        """Update generator parameters from sliders"""
        self.generator.load_current = self.load_current_slider.value
        self.generator.power_factor = self.load_power_slider.value
        self.generator.If = self.field_current_slider.value
        self.generator.omega_m = 2 * np.pi * self.speed_slider.value / 60
        self.generator.ambient_temp = self.ambient_temp_slider.value
        self.sim_method = self.solver_dropdown.value

    def on_start(self, button):
        """Start simulation"""
        self.is_running = True
        self.start_button.disabled = True
        self.stop_button.disabled = False

        self.run_simulation()

    def on_stop(self, button):
        """Stop simulation"""
        self.is_running = False
        self.start_button.disabled = False
        self.stop_button.disabled = True

    def on_reset(self, button):
        """Reset simulation"""
        self.is_running = False
        self.generator.reset_state()
        self.current_time = 0
        self.start_button.disabled = False
        self.stop_button.disabled = True

        # Clear outputs
        with self.plot_output:
            clear_output(wait=True)

        with self.output_display:
            clear_output(wait=True)
            print("✅ Simulation reset. Press START to begin.")

    def on_calculate(self, button):
        """Calculate and display theoretical solution"""
        self.display_theoretical_solution()

    def display_theoretical_solution(self):
        """Display Example 7.3 theoretical solution"""
        with self.output_display:
            clear_output(wait=True)
            self.solution.print_results()

    def run_simulation(self):
        """Run the multi-physics simulation"""
        self.update_generator_parameters()

        # Set up simulation
        t_span = (0, self.sim_time_slider.value)
        Tm = self.torque_slider.value
        If = self.field_current_slider.value
        pf = self.load_power_slider.value

        # Run simulation
        with self.output_display:
            clear_output(wait=True)
            print(f"🚀 Running {self.sim_method} simulation...")
            print(f"   Time span: {t_span[1]:.1f} seconds")
            print(f"   Mechanical torque: {Tm:.1f} Nm")
            print(f"   Field current: {If:.2f} A")
            print(f"   Load power factor: {pf:.2f}")
            print(f"   Load current: {self.load_current_slider.value:.2f} A")

        try:
            t_eval, y_sol = self.generator.simulate_dynamics(
                t_span, Tm, If, pf, method=self.sim_method, dt=0.01
            )

            with self.output_display:
                print("\n✅ Simulation completed successfully!")
                self.display_simulation_summary()

            # Update plots
            self.update_plots()

            # Update economic analysis
            self.update_economic_analysis()

            # Update loss breakdown
            self.update_loss_breakdown()

        except Exception as e:
            with self.output_display:
                print(f"\n❌ Simulation error: {str(e)}")

        finally:
            self.is_running = False
            self.start_button.disabled = False
            self.stop_button.disabled = True

    def display_simulation_summary(self):
        """Display summary of simulation results"""
        if len(self.generator.history['time']) == 0:
            return

        # Final values
        T_stator_final = self.generator.history['T_stator'][-1]
        T_rotor_final = self.generator.history['T_rotor'][-1]
        omega_final = self.generator.history['omega_m'][-1]
        Te_final = self.generator.history['Te'][-1]
        efficiency_final = self.generator.history['efficiency'][-1]
        P_out_final = self.generator.history['P_out'][-1]

        print("\n" + "="*70)
        print("SIMULATION SUMMARY")
        print("="*70)
        print(f"Final stator temperature:    {T_stator_final:.2f} °C")
        print(f"Final rotor temperature:     {T_rotor_final:.2f} °C")
        print(f"Final speed:                 {omega_final * 60 / (2*np.pi):.1f} rpm")
        print(f"Final electromagnetic torque: {Te_final:.2f} Nm")
        print(f"Final output power:          {P_out_final/1000:.2f} kW")
        print(f"Final efficiency:            {efficiency_final:.2f} %")
        print("="*70)

    def update_plots(self):
        """Update dynamic visualization plots"""
        if len(self.generator.history['time']) == 0:
            return

        with self.plot_output:
            clear_output(wait=True)

            fig, axes = plt.subplots(3, 2, figsize=(14, 10))
            fig.suptitle('Multi-Physics Simulation Results', fontsize=16, fontweight='bold')

            t = self.generator.history['time']

            # Plot 1: Currents
            axes[0, 0].plot(t, self.generator.history['id'], 'b-', label='Id (A)', linewidth=2)
            axes[0, 0].plot(t, self.generator.history['iq'], 'r-', label='Iq (A)', linewidth=2)
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Current (A)')
            axes[0, 0].set_title('D-Q Axis Currents')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

            # Plot 2: Torque
            axes[0, 1].plot(t, self.generator.history['Te'], 'g-', label='Te (Nm)', linewidth=2)
            axes[0, 1].plot(t, self.generator.history['Tm'], 'orange', linestyle='--',
                           label='Tm (Nm)', linewidth=2)
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Torque (Nm)')
            axes[0, 1].set_title('Electromagnetic & Mechanical Torque')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)

            # Plot 3: Speed
            rpm = np.array(self.generator.history['omega_m']) * 60 / (2 * np.pi)
            axes[1, 0].plot(t, rpm, 'purple', linewidth=2)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Speed (rpm)')
            axes[1, 0].set_title('Rotor Speed')
            axes[1, 0].grid(True, alpha=0.3)
            axes[1, 0].axhline(y=self.solution.nn, color='r', linestyle='--',
                              label='Nominal', alpha=0.5)
            axes[1, 0].legend()

            # Plot 4: Temperature
            axes[1, 1].plot(t, self.generator.history['T_stator'], 'red',
                           label='Stator (°C)', linewidth=2)
            axes[1, 1].plot(t, self.generator.history['T_rotor'], 'blue',
                           label='Rotor (°C)', linewidth=2)
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Temperature (°C)')
            axes[1, 1].set_title('Thermal Behavior')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)

            # Plot 5: Power & Efficiency
            ax5 = axes[2, 0]
            ax5_twin = ax5.twinx()
            line1 = ax5.plot(t, np.array(self.generator.history['P_out'])/1000,
                            'b-', label='Power Out (kW)', linewidth=2)
            line2 = ax5_twin.plot(t, self.generator.history['efficiency'],
                                 'r-', label='Efficiency (%)', linewidth=2)
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Power (kW)', color='b')
            ax5_twin.set_ylabel('Efficiency (%)', color='r')
            ax5.set_title('Output Power & Efficiency')
            ax5.tick_params(axis='y', labelcolor='b')
            ax5_twin.tick_params(axis='y', labelcolor='r')
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax5.legend(lines, labels, loc='best')
            ax5.grid(True, alpha=0.3)

            # Plot 6: EMF & Field
            ax6 = axes[2, 1]
            ax6_twin = ax6.twinx()
            line1 = ax6.plot(t, self.generator.history['E0'], 'g-',
                            label='EMF E0 (V)', linewidth=2)
            line2 = ax6_twin.plot(t, self.generator.history['If'], 'm-',
                                 label='If (A)', linewidth=2)
            ax6.set_xlabel('Time (s)')
            ax6.set_ylabel('EMF (V)', color='g')
            ax6_twin.set_ylabel('Field Current (A)', color='m')
            ax6.set_title('Excitation EMF & Field Current')
            ax6.tick_params(axis='y', labelcolor='g')
            ax6_twin.tick_params(axis='y', labelcolor='m')
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax6.legend(lines, labels, loc='best')
            ax6.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.show()

    def update_economic_analysis(self):
        """Update economic analysis display"""
        with self.economic_output:
            clear_output(wait=True)

            if len(self.generator.history['time']) == 0:
                print("⚠️ No simulation data available. Run simulation first.")
                return

            print("="*70)
            print("ECONOMIC ANALYSIS")
            print("="*70)

            # Annual operating hours scenarios
            scenarios = [2000, 4000, 6000, 8760]  # hours

            for hours in scenarios:
                print(f"\n📊 Scenario: {hours} operating hours/year")
                print("-"*70)

                cost_annual, E_loss_annual = self.economics.calculate_energy_cost(hours)
                lcoe, pv_costs, pv_energy = self.economics.calculate_lcoe(hours)
                payback = self.economics.calculate_payback_period(0.15)  # $0.15/kWh revenue

                print(f"   Annual energy loss:        {E_loss_annual:.2f} kWh")
                print(f"   Annual energy cost:        ${cost_annual:.2f}")
                print(f"   Total maintenance cost:    ${self.economics.maintenance_cost_annual:.2f}/year")
                print(f"   LCOE:                      ${lcoe:.4f}/kWh")
                print(f"   PV of total costs:         ${pv_costs:.2f}")
                print(f"   PV of energy produced:     {pv_energy:.2f} kWh")
                print(f"   Simple payback period:     {payback:.2f} years")

            # Summary
            print("\n" + "="*70)
            print("COST BREAKDOWN (per year at 4000 hours)")
            print("="*70)
            cost_4000, loss_4000 = self.economics.calculate_energy_cost(4000)
            print(f"   Capital cost (amortized):  ${self.economics.capital_cost / self.economics.lifetime_years:.2f}")
            print(f"   Energy loss cost:          ${cost_4000:.2f}")
            print(f"   Maintenance cost:          ${self.economics.maintenance_cost_annual:.2f}")
            print(f"   Total annual cost:         ${self.economics.capital_cost / self.economics.lifetime_years + cost_4000 + self.economics.maintenance_cost_annual:.2f}")
            print("="*70)

    def update_loss_breakdown(self):
        """Update loss breakdown display"""
        with self.loss_output:
            clear_output(wait=True)

            if len(self.generator.history['time']) == 0:
                print("⚠️ No simulation data available. Run simulation first.")
                return

            # Average losses
            P_copper_avg = np.mean(self.generator.history['P_copper'])
            P_iron_avg = np.mean(self.generator.history['P_iron'])
            P_friction_avg = np.mean(self.generator.history['P_friction'])
            P_stray_avg = np.mean(self.generator.history['P_stray'])
            P_total_loss = P_copper_avg + P_iron_avg + P_friction_avg + P_stray_avg

            P_out_avg = np.mean(self.generator.history['P_out'])
            efficiency_avg = np.mean(self.generator.history['efficiency'])

            print("="*70)
            print("LOSS BREAKDOWN & EFFICIENCY ANALYSIS")
            print("="*70)

            print("\n📉 AVERAGE LOSSES:")
            print(f"   Copper losses (I²R):       {P_copper_avg:.2f} W  ({P_copper_avg/P_total_loss*100:.1f}%)")
            print(f"   Iron/Core losses:          {P_iron_avg:.2f} W  ({P_iron_avg/P_total_loss*100:.1f}%)")
            print(f"   Mechanical friction:       {P_friction_avg:.2f} W  ({P_friction_avg/P_total_loss*100:.1f}%)")
            print(f"   Stray load losses:         {P_stray_avg:.2f} W  ({P_stray_avg/P_total_loss*100:.1f}%)")
            print(f"   TOTAL LOSSES:              {P_total_loss:.2f} W")

            print("\n⚡ POWER FLOW:")
            P_in_avg = P_out_avg + P_total_loss
            print(f"   Input power:               {P_in_avg/1000:.2f} kW")
            print(f"   Output power:              {P_out_avg/1000:.2f} kW")
            print(f"   Total losses:              {P_total_loss/1000:.2f} kW")
            print(f"   Average efficiency:        {efficiency_avg:.2f} %")

            print("\n📊 LOSS DISTRIBUTION:")
            print(f"   {'Loss Type':<25} {'Power (W)':<15} {'Percentage'}")
            print("-"*70)
            print(f"   {'Copper':<25} {P_copper_avg:<15.2f} {'█' * int(P_copper_avg/P_total_loss*50)}")
            print(f"   {'Iron':<25} {P_iron_avg:<15.2f} {'█' * int(P_iron_avg/P_total_loss*50)}")
            print(f"   {'Friction':<25} {P_friction_avg:<15.2f} {'█' * int(P_friction_avg/P_total_loss*50)}")
            print(f"   {'Stray':<25} {P_stray_avg:<15.2f} {'█' * int(P_stray_avg/P_total_loss*50)}")

            # Temperature derating
            T_max = max(self.generator.history['T_stator'][-1],
                       self.generator.history['T_rotor'][-1])
            T_limit = 155  # Class F insulation
            derating = max(0, (T_limit - T_max) / T_limit * 100)

            print("\n🌡️ THERMAL DERATING:")
            print(f"   Maximum temperature:       {T_max:.1f} °C")
            print(f"   Temperature limit (Class F): {T_limit} °C")
            print(f"   Thermal margin:            {T_limit - T_max:.1f} °C")
            print(f"   Available derating:        {derating:.1f} %")

            if T_max > T_limit:
                print("   ⚠️ WARNING: Temperature exceeds safe limit!")

            print("="*70)

    def display(self):
        """Display the GUI"""
        display(self.main_widget)


# =============================================================================
# MAIN EXECUTION
# =============================================================================

def main():
    """Main function to run the simulator"""

    print("\n" + "="*80)
    print(" "*20 + "INITIALIZING ADVANCED GENERATOR SIMULATOR")
    print("="*80 + "\n")

    # Create and display theoretical solution
    solution = Example73Solution()
    solution.print_results()

    print("\n" + "="*80)
    print(" "*25 + "LAUNCHING INTERACTIVE LAB")
    print("="*80 + "\n")

    # Create and display GUI
    simulator = AdvancedGeneratorSimulatorGUI()
    simulator.display()

    return simulator


# Run the simulator
if __name__ == "__main__":
    simulator = main()
