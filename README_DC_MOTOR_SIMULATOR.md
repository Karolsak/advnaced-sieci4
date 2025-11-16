# Advanced DC Motor Multi-Physics Simulator

A comprehensive DC motor simulation tool with multi-physics analysis, including electromagnetic, thermal, and mechanical modeling.

## Features

### 1. Problem Solutions Tab
- **Problem 3**: DC shunt motor with flux reduction (75% of normal)
  - Case (a): Torque unchanged → Speed = 1042 rpm ✓
  - Case (b): Torque reduced by 20% → Speed = 1061 rpm ✓
- **Problem 4**: DC shunt motor with flux increase (120% of initial)
  - New armature current and speed calculation
  - All solutions verified with expected results

### 2. Motor Simulation Tab
#### Motor Types
- **Shunt Motor**: Separate field and armature circuits
- **Series Motor**: Field and armature in series

#### ODE Solvers
- **RK45 (Runge-Kutta)**: High-accuracy adaptive solver
- **Euler Method**: Simple first-order solver for comparison

#### Real-Time Controls
- **Supply Voltage**: 0-500 V (adjustable slider)
- **Load Torque**: 0-50 N.m (adjustable slider)
- **Armature Resistance**: 0.1-5 Ω
- **Moment of Inertia**: 0.001-0.1 kg.m²

#### Dynamic Visualization
Six real-time graphs:
1. **Speed (rpm)**: Motor rotational speed
2. **Currents (A)**: Armature and field currents
3. **Torque (N.m)**: Electromagnetic torque vs load torque
4. **Temperature (°C)**: Armature and field temperatures with thermal limits
5. **Power (W)**: Input and output power
6. **Losses (W)**: Total power losses

#### Status Display
Real-time monitoring:
- Simulation time
- Current speed
- Armature current
- Electromagnetic torque
- Output power
- Temperature

### 3. Thermal Analysis Tab
Multi-physics thermal modeling:
- **Temperature Distribution**: Armature and field temperatures over time
- **Heat Generation vs Dissipation**: Thermal balance analysis
- **Thermal Derating Factor**: Performance reduction at high temperatures
- **Temperature vs Current**: Correlation analysis

#### Thermal Model Features
- Thermal resistance and capacitance modeling
- Coupled electromagnetic-thermal equations
- Automatic derating when temperature exceeds 100°C
- Maximum temperature protection (130°C)

### 4. Loss Analysis Tab
Detailed loss breakdown:
- **Copper Losses**: I²R losses in armature and field
- **Iron Losses**: Hysteresis and eddy current losses
- **Mechanical Losses**: Friction and windage
- **Stray Load Losses**: Additional losses under load

#### Loss Visualizations
1. **Pie Chart**: Loss distribution breakdown
2. **Losses vs Time**: Time-domain loss analysis
3. **Efficiency vs Load**: Performance characteristics
4. **Cumulative Energy Loss**: Total energy wasted

### 5. Economic Analysis Tab
Comprehensive cost analysis:
- **Operating Costs**: Energy consumption and costs
- **Lifetime Costs**: Total cost of ownership
- **Efficiency Analysis**: Performance vs cost
- **Cost per Operating Hour**: Economic efficiency metric

#### Configurable Parameters
- Electricity cost ($/kWh)
- Operating hours per day
- Operating days per year
- Motor initial cost
- Annual maintenance cost
- Expected lifetime (years)

### 6. Mechanical Stress Tab
Mechanical stress analysis:
- **Shaft Shear Stress**: Torsional stress in shaft
- **Bearing Load**: Force on bearings
- **Torque-Speed Characteristic**: Performance curve
- **Stress Distribution**: Radial stress profile

## Mathematical Models

### Electrical Equations (Shunt Motor)
```
dIa/dt = (V - Eb - Ia*Ra) / La
dIf/dt = (V - If*Rf) / Lf
Eb = Kb * ω * φ
```

### Mechanical Equations
```
Te = Kt * φ * Ia * derating_factor
dω/dt = (Te - B*ω - TL) / J
```

### Thermal Equations
```
dT_arm/dt = (P_copper_a + P_stray - (T_arm - T_ambient)/R_th_a) / C_th_a
dT_field/dt = (P_copper_f - (T_field - T_ambient)/R_th_f) / C_th_f
```

### Loss Calculations
```
P_copper = I²R (armature and field)
P_iron = k_iron * (ω/(2π))²
P_mechanical = k_mech * ω²
P_stray = k_stray * (Ia² + If²)
```

## Installation

### Requirements
```bash
pip install numpy scipy matplotlib tkinter
```

Note: `tkinter` usually comes with Python installation.

## Usage

### Running the Simulator
```bash
python3 dc_motor_simulator.py
```

### Basic Workflow
1. **Select Motor Type**: Choose between Shunt or Series motor
2. **Select Solver**: Choose RK45 or Euler method
3. **Adjust Parameters**: Use sliders to set voltage, load torque, resistance, and inertia
4. **Start Simulation**: Click "Start" button
5. **Monitor Results**: Watch real-time graphs and status display
6. **Analyze**: Switch between tabs for detailed analysis
7. **Stop/Reset**: Use control buttons as needed

### Economic Analysis
1. Navigate to "Economic Analysis" tab
2. Enter economic parameters (electricity cost, operating hours, etc.)
3. Click "Calculate Economics"
4. Review annual and lifetime costs

## Key Features for Electrical Engineering

### 1. Multi-Physics Coupling
- Simultaneous solution of electrical, thermal, and mechanical equations
- Accurate temperature-dependent performance prediction
- Real-world derating behavior

### 2. Advanced Control Methods
- Parameter adjustment during simulation
- Real-time response to load changes
- Multiple solver options for accuracy vs speed

### 3. Practical Applications
- Motor selection and sizing
- Energy efficiency analysis
- Cost optimization
- Thermal management design
- Performance prediction under various conditions

### 4. Educational Value
- Visual understanding of motor dynamics
- Comparison of different motor types
- Impact of parameters on performance
- Economic decision-making tools

## Technical Specifications

### Default Parameters
- Supply Voltage: 220 V
- Armature Resistance: 0.5 Ω
- Armature Inductance: 0.01 H
- Field Resistance: 200 Ω
- Field Inductance: 10 H
- Back EMF Constant: 0.8 V.s/rad
- Torque Constant: 0.8 N.m/A
- Moment of Inertia: 0.02 kg.m²
- Viscous Friction: 0.001 N.m.s/rad

### Thermal Parameters
- Armature Thermal Resistance: 2.0 K/W
- Armature Thermal Capacitance: 50 J/K
- Field Thermal Resistance: 3.0 K/W
- Field Thermal Capacitance: 80 J/K
- Ambient Temperature: 25°C
- Maximum Temperature: 130°C

## Validation

### Problem 3 Results
- Case (a) Speed: 1041.60 rpm (Expected: 1042 rpm) ✓
- Case (b) Speed: 1061.20 rpm (Expected: 1061 rpm) ✓

### Problem 4 Results
- Calculations match theoretical expectations
- Speed and current relationships verified

## Auto-Scaling Features
- Automatic window resizing support
- Responsive graph layouts
- Dynamic canvas adjustment
- Grid-based responsive design

## Tips for Best Results

1. **Start with default parameters** to understand baseline behavior
2. **Use RK45 solver** for accurate results
3. **Monitor temperature** to avoid thermal limits
4. **Adjust load gradually** to observe transient response
5. **Compare motor types** under same conditions
6. **Run economic analysis** after steady-state simulation

## Troubleshooting

- **Simulation unstable**: Reduce time step or use RK45 solver
- **Temperature too high**: Reduce voltage or load torque
- **Slow performance**: Use Euler method or increase time step
- **Graph not updating**: Ensure simulation is running

## Future Enhancements
- PWM control simulation
- Field weakening control
- Regenerative braking
- Multiple motor comparison
- Export data to CSV
- Custom parameter profiles

## License
Educational and research use.

## Author
Advanced DC Motor Simulator - Multi-Physics Analysis Tool
