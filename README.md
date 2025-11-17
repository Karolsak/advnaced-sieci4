# Advanced DC Motor Multi-Physics Simulator

A comprehensive application for DC motor analysis, simulation, and educational problem-solving with advanced multi-physics modeling.

## 🚀 Multiple Versions Available

### 1. **🌐 Web Version (HTML)** - `dc_motor_simulator.html`
   - No installation required - runs directly in any modern browser
   - Beautiful responsive design with gradient UI
   - All features included (6 interactive tabs)
   - Real-time Chart.js visualizations
   - Perfect for quick access and demonstrations

### 2. **🖥️ Desktop Version (Python)** - `dc_motor_simulator.py`
   - Full-featured Tkinter application
   - More detailed analysis capabilities
   - Requires Python and dependencies

### 3. **⚡ Shunt Motor Speed Control Simulator** - `shunt_motor_speed_control_advanced.py`
   - **NEW! Advanced shunt motor speed control analysis**
   - **Solves Example 30.25** with detailed explanation
   - **Multi-physics simulation**: Electromagnetic-Thermal-Mechanical coupling
   - **Real-time ODE solvers**: RK45 and Euler methods
   - **5 Interactive Tabs**:
     - Main Control: Example 30.25 solver and parameter adjustment
     - Dynamic Simulation: Real-time simulation with 4 graphs
     - Loss Analysis: Detailed loss breakdown and torque analysis
     - Economic Analysis: Operating costs and projections
     - Thermal & Derating: Temperature monitoring and derating factors
   - **Advanced Features**:
     - Series resistance control (0-20Ω)
     - Load torque adjustment (0-50 N.m)
     - Detailed loss models (copper, iron, mechanical, stray)
     - Thermal time constant analysis
     - Cost per hour/day/month/year calculations
     - Auto-scaling responsive interface
   - See [SHUNT_MOTOR_README.md](SHUNT_MOTOR_README.md) for detailed documentation

### 4. **🔌 Salient-Pole Synchronous Generator Simulator** - `example_7_3_advanced_simulator.py`
   - **NEW! Advanced synchronous generator analysis and simulation**
   - **Solves Example 7.3** - Salient-pole synchronous generator with slip test
   - **Multi-Physics Simulation**: Electromagnetic-Thermal-Mechanical coupling
   - **Theoretical Solution**:
     - Synchronous reactances: Xsd = 10.177 Ω, Xsq = 5.180 Ω
     - Nominal field current: Ifn = 14.089 A
     - Field voltage at 120°C: Vf = 16.547 V
   - **Interactive Features**:
     - Real-time parameter adjustment (power factor, load current, field current, speed, torque)
     - Multiple ODE solvers (RK45, Euler)
     - 4 Interactive Tabs: Controls, Results, Economics, Losses
     - Auto-scaling dynamic visualization (6 simultaneous plots)
   - **Advanced Models**:
     - D-Q axis representation with salient-pole effects
     - Coupled thermal dynamics (stator and rotor)
     - Mechanical shaft dynamics with inertia and damping
     - Comprehensive loss breakdown (copper, iron, friction, stray)
   - **Economic Analysis**:
     - Levelized Cost of Energy (LCOE)
     - Annual operating costs
     - Present value calculations
     - Multi-scenario analysis (2000-8760 hours/year)
   - **Jupyter Notebook Version**: `example_7_3_interactive_lab.ipynb`
     - Better browser compatibility
     - Step-by-step educational format
     - Enhanced visualization
   - See [README_EXAMPLE_7_3.md](README_EXAMPLE_7_3.md) for detailed documentation

## Problem 7 Solution

### Problem Statement
A 250 V shunt motor with an armature resistance of 0.5 Ω and a shunt field resistance of 250 Ω drives a load whose torque remains constant. The motor draws a line current of 21 A when the speed is 600 rpm. If the speed is to be raised to 800 rpm, what change must be affected in the shunt field resistance? Assume that the magnetization curve of the motor is a straight line.

### Solution

**Given Data:**
- Supply Voltage (V) = 250 V
- Armature Resistance (Ra) = 0.5 Ω
- Initial Field Resistance (Rf₁) = 250 Ω
- Initial Line Current (IL₁) = 21 A
- Initial Speed (N₁) = 600 rpm
- Target Speed (N₂) = 800 rpm
- Conditions: Constant load torque, Linear magnetization curve

**Initial Operating Point:**
- Field Current (If₁) = V/Rf₁ = 250/250 = 1.000 A
- Armature Current (Ia₁) = IL₁ - If₁ = 21 - 1 = 20.000 A
- Back EMF (Eb₁) = V - Ia₁×Ra = 250 - 20×0.5 = 240.00 V

**Analysis:**

For a DC shunt motor with linear magnetization curve:
1. Flux is proportional to field current: Φ ∝ If
2. For constant torque: T = k×Φ×Ia → If₁×Ia₁ = If₂×Ia₂
3. Back EMF: Eb = K×Φ×N = K×If×N
4. Therefore: Eb₁/(If₁×N₁) = Eb₂/(If₂×N₂)

This leads to a quadratic equation:
```
320×If₂² - 250×If₂ + 10 = 0
```

**Final Operating Point (at 800 rpm):**
- New Field Current (If₂) = 0.7390 A
- New Armature Current (Ia₂) = 27.07 A
- New Line Current (IL₂) = 27.80 A
- New Back EMF (Eb₂) = 236.47 V

**ANSWER:**
- **New Field Resistance (Rf₂) = 338.31 Ω**
- **Change in Field Resistance (ΔRf) = +88.31 Ω**
- **The field resistance must be INCREASED by 88.31 Ω (by adding external resistance in series with the field winding)**

**Verification:**
- Torque ratio check: If₁×Ia₁ / If₂×Ia₂ = 1.0000 ✓
- Speed ratio from Eb: 1.3333
- Expected speed ratio: 800/600 = 1.3333 ✓

## Features

### 1. User Interface (Tkinter GUI)
- **Multi-tab interface** with organized functionality
- **Problem Solutions tab**: Displays theoretical problem solutions
- **Motor Simulation tab**: Real-time dynamic simulation
- **Thermal Analysis tab**: Temperature distribution and derating analysis
- **Loss Analysis tab**: Detailed breakdown of all losses
- **Economic Analysis tab**: Cost analysis and ROI calculations
- **Mechanical Stress tab**: Shaft stress and bearing load analysis

### 2. Control Panel
- **Motor Type Selection**: Shunt or Series motor
- **ODE Solver Selection**: RK45 (Runge-Kutta) or Euler method
- **Adjustable Parameters**:
  - Supply Voltage (0-500V)
  - Load Torque (0-50 N.m)
  - Armature Resistance (0.1-5 Ω)
  - Moment of Inertia (0.001-0.1 kg.m²)
- **Control Buttons**: Start, Stop, Reset
- **Real-time Status Display**: Time, Speed, Current, Torque, Power, Temperature

### 3. Mathematical Modeling

#### Circuit Model - Differential Equations
**Shunt Motor** (6 state variables):
```python
State: [Ia, If, ω, θ, T_arm, T_field]

dIa/dt = (V - Eb - Ia×Ra) / La
dIf/dt = (V - If×Rf) / Lf
dω/dt = (Te - B×ω - TL) / J
dθ/dt = ω
dT_arm/dt = (P_copper_a + P_stray - (T_arm - T_ambient)/R_th_a) / C_th_a
dT_field/dt = (P_copper_f - (T_field - T_ambient)/R_th_f) / C_th_f
```

**Series Motor** (4 state variables):
```python
State: [Ia, ω, θ, T_arm]

dIa/dt = (V - Eb - Ia×(Ra+Rf)) / (La+Lf)
dω/dt = (Te - B×ω - TL) / J
dθ/dt = ω
dT_arm/dt = (P_copper - (T_arm - T_ambient)/R_th_a) / C_th_a
```

### 4. Multi-Physics Simulation

#### Electromagnetic Model
- Back EMF calculation: Eb = Kb×ω×If
- Electromagnetic torque: Te = Kt×If×Ia (shunt) or Te = Kt×Ia² (series)
- Temperature derating factor applied to torque

#### Thermal Model
- **Coupled thermal-electrical equations**
- **Heat sources**:
  - Copper losses (armature and field)
  - Iron losses (proportional to speed²)
  - Mechanical friction losses
  - Stray load losses
- **Thermal network**: Resistance-capacitance model
- **Temperature limits**: 130°C maximum with derating above 100°C

#### Mechanical Model
- **Shaft stress analysis**:
  - Torsional shear stress: τ = T×r/J_polar
  - Bearing load calculation
- **Dynamic response**: J×dω/dt = Te - B×ω - TL

### 5. Loss Analysis

Detailed breakdown of all losses:
1. **Copper Losses**:
   - Armature: Ia²×Ra
   - Field: If²×Rf
2. **Iron Losses**: k_iron×(ω/2π)²
3. **Mechanical Losses**: k_mech×ω²
4. **Stray Load Losses**: k_stray×(Ia² + If²)

### 6. Visualization

#### Real-time Graphs (6 plots):
1. Speed (rpm) vs Time
2. Armature and Field Currents vs Time
3. Electromagnetic Torque vs Time (with load torque reference)
4. Temperature (Armature and Field) vs Time
5. Input/Output Power vs Time
6. Total Losses vs Time

#### Thermal Analysis Plots (4 plots):
- Temperature distribution
- Heat generation vs dissipation
- Thermal derating factor
- Temperature vs Current relationship

#### Loss Analysis Plots (4 plots):
- Loss breakdown pie chart
- Losses vs Time
- Efficiency vs Load curve
- Cumulative energy loss

#### Mechanical Stress Plots (4 plots):
- Shaft shear stress vs Time
- Bearing load vs Time
- Torque-speed characteristic
- Stress distribution

### 7. Economic Analysis

**Input Parameters**:
- Electricity cost ($/kWh)
- Operating hours per day
- Operating days per year
- Motor initial cost ($)
- Maintenance cost per year ($)
- Expected lifetime (years)

**Calculated Metrics**:
- Annual energy consumption and cost
- Lifetime total cost (capital + energy + maintenance)
- Cost per operating hour
- Efficiency analysis
- Cost of losses

### 8. Advanced Features

- **Auto-scaling**: Automatic window and graph resize
- **Dual ODE Solvers**:
  - RK45: 4th-order Runge-Kutta (adaptive step)
  - Euler: Simple first-order method
- **Real-time simulation**: 10ms update rate
- **Data buffering**: Efficient deque-based storage (1000 points)
- **Temperature derating**: Automatic power reduction at high temperatures
- **RMS values**: All electrical quantities use RMS values

## Installation

### Requirements
```bash
pip install numpy scipy matplotlib
```

For GUI support (required):
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# Fedora
sudo dnf install python3-tkinter

# macOS (usually included)
# Windows (usually included with Python)
```

## Usage

### Option 1: Web Browser (HTML Version)
Simply open `dc_motor_simulator.html` in any modern web browser:
```bash
# Linux/Mac
open dc_motor_simulator.html

# Windows
start dc_motor_simulator.html

# Or just double-click the file
```

**Features of HTML Version:**
- ✅ No installation required
- ✅ Runs directly in browser
- ✅ All features included (6 interactive tabs)
- ✅ Beautiful responsive design
- ✅ Real-time Chart.js visualizations
- ✅ Works on desktop, tablet, and mobile

### Option 2: Python Desktop Application
```bash
python3 dc_motor_simulator.py
```

### Running Problem 7 Calculation Only
```bash
python3 test_problem7.py
```

### Using the Simulator

1. **Select Motor Type**: Choose between Shunt or Series motor
2. **Select ODE Solver**: Choose RK45 (more accurate) or Euler (faster)
3. **Adjust Parameters**: Use sliders to set voltage, load torque, resistance, and inertia
4. **Start Simulation**: Click "Start" to begin real-time simulation
5. **Monitor Results**: Watch real-time graphs and status display
6. **Economic Analysis**: Switch to Economic Analysis tab and click "Calculate Economics"
7. **Stop/Reset**: Use Stop to pause, Reset to clear and restart

## Technical Details

### Electrical Parameters (Default)
- Armature Resistance (Ra): 0.5 Ω
- Armature Inductance (La): 0.01 H
- Field Resistance (Rf): 200 Ω
- Field Inductance (Lf): 10 H
- Back EMF Constant (Kb): 0.8 V.s/rad
- Torque Constant (Kt): 0.8 N.m/A

### Mechanical Parameters (Default)
- Moment of Inertia (J): 0.02 kg.m²
- Viscous Friction (B): 0.001 N.m.s/rad
- Load Torque (TL): Adjustable 0-50 N.m

### Thermal Parameters (Default)
- Armature Thermal Resistance: 2.0 K/W
- Armature Thermal Capacitance: 50 J/K
- Field Thermal Resistance: 3.0 K/W
- Field Thermal Capacitance: 80 J/K
- Ambient Temperature: 25°C
- Maximum Temperature: 130°C

### Loss Coefficients (Default)
- Iron Loss Coefficient: 0.5
- Mechanical Loss Coefficient: 0.02
- Stray Loss Coefficient: 0.01

## Educational Value

This simulator is designed for:
- **Electrical Engineering Students**: Understanding DC motor behavior
- **Control Systems**: Studying dynamic response and control strategies
- **Thermal Management**: Analyzing temperature effects on performance
- **Economic Analysis**: Understanding operational costs
- **Research**: Multi-physics coupled simulations

## Solved Problems

The application includes solutions to the following theoretical problems:

1. **Problem 3**: DC shunt motor with flux reduction (75%)
   - Case (a): Torque unchanged → Speed: 1042 rpm
   - Case (b): Torque reduced by 20% → Speed: 1061 rpm

2. **Problem 4**: DC shunt motor with flux increase (120%)
   - New current: 23.33 A
   - New speed: 696 rpm

3. **Problem 7**: Field resistance change for speed control
   - Speed increase: 600 → 800 rpm
   - Field resistance increase: +88.31 Ω

4. **Example 30.25**: Armature resistance control for speed reduction
   - Initial: 240V, 15A, 800 rpm
   - Question 1: Resistance needed to reduce speed to 400 rpm
     - **Answer: R_added = 7.70 Ω**
   - Question 2: Speed when torque is halved with this resistance
     - **Answer: Speed = 615.58 rpm**
   - Uses armature resistance control method
   - Demonstrates speed-torque relationships

## Mathematical Background

### Speed Control Methods
1. **Field Control**: Varying field resistance (used in Problem 7)
   - Increases speed by weakening field flux
   - Most efficient for above-rated speed

2. **Armature Voltage Control**: Varying supply voltage
   - Linear speed control below rated speed

3. **Armature Resistance Control**: Adding external resistance
   - Poor efficiency, rarely used

### Key Equations

**Speed-Flux Relationship**:
```
N ∝ Eb/Φ = (V - Ia×Ra)/Φ
```

**Torque-Current Relationship**:
```
T ∝ Φ×Ia
```

**For Constant Torque and Linear Magnetization**:
```
If₁×Ia₁ = If₂×Ia₂
```

## License

This educational software is provided for learning and research purposes.

## Authors

Created for advanced electrical engineering education and practical motor analysis.

## References

1. Electrical Machinery Fundamentals - Stephen J. Chapman
2. Electric Machinery - A.E. Fitzgerald
3. Power Electronics and Motor Drives - Bimal K. Bose
