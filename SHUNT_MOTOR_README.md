# Advanced DC Shunt Motor Speed Control Simulator

## Overview

This is a comprehensive Python application for simulating and analyzing DC shunt motor speed control with multi-physics modeling. It solves Example 30.25 and provides advanced features for electrical engineering analysis.

## Example 30.25 Solution

### Problem Statement
A 240V shunt motor has an armature current of 15A when running at 800 rpm against full load torque. The armature resistance is 0.6Ω.

**Question 1:** What resistance must be inserted in series with the armature to reduce the speed to 400 rpm at the same torque?

**Question 2:** What will be the speed if the load torque is halved with this resistance in the circuit?

### Solution

**Answer 1:** R_added = **7.70 Ω**

**Calculation:**
- Initial Back EMF: Eb1 = V - Ia × Ra = 240 - 15 × 0.6 = 231 V
- At 400 rpm with same torque (same Ia = 15A):
- Eb2 = Eb1 × (N2/N1) = 231 × (400/800) = 115.5 V
- From voltage equation: 240 = 115.5 + 15 × (0.6 + R_added)
- R_added = (240 - 115.5)/15 - 0.6 = 7.70 Ω

**Answer 2:** Speed = **615.58 rpm**

**Calculation:**
- Torque halved means Ia halved: Ia3 = 7.5 A
- Total resistance: R_total = 0.6 + 7.70 = 8.3 Ω
- Back EMF: Eb3 = 240 - 7.5 × 8.3 = 177.75 V
- Speed: N3 = 800 × (177.75/231) = 615.58 rpm

## Features

### 1. Main Control Tab
- **Example 30.25 Solver**: Automatically solves the speed control problem
- **Motor Parameters**: Adjustable parameters with real-time sliders
  - Supply voltage (0-500V)
  - Armature resistance (0.1-10Ω)
  - Field resistance (10-500Ω)
  - Inductances (armature and field)
  - Mechanical parameters (inertia, friction)

### 2. Dynamic Simulation Tab
- **Real-time ODE Solvers**:
  - Runge-Kutta 45 (RK45) - High accuracy
  - Euler Method - Fast computation
- **Control Inputs**:
  - Added series resistance (0-20Ω)
  - Load torque (0-50 N.m)
- **Real-time Graphs**:
  - Speed vs Time
  - Armature Current vs Time
  - Input Power vs Time
  - Efficiency vs Time
- **Control Buttons**: Start, Stop, Reset

### 3. Loss Analysis Tab
- **Detailed Loss Breakdown**:
  - Copper losses (armature + field)
  - Iron losses (hysteresis + eddy current)
  - Mechanical losses (friction + windage)
  - Stray load losses
- **Visualizations**:
  - Pie chart of loss distribution
  - Electromagnetic torque analysis
  - Loss components over time

### 4. Economic Analysis Tab
- **Cost Parameters**:
  - Electricity cost ($/kWh)
  - Maintenance cost ($/hour)
- **Operating Metrics**:
  - Total energy consumed (kWh)
  - Total operating time (hours)
  - Energy costs
  - Maintenance costs
  - Total operating cost
- **Projections**:
  - Cost per hour/day/month/year
  - Cumulative cost over time graph

### 5. Thermal & Derating Tab
- **Thermal Parameters**:
  - Ambient temperature
  - Thermal resistance (°C/W)
  - Thermal capacitance (J/°C)
  - Maximum temperature limit
- **Analysis**:
  - Real-time temperature monitoring
  - Temperature rise calculation
  - Thermal margin
  - Automatic derating factor calculation
  - Temperature status indicators
  - Thermal time constant

## Multi-Physics Modeling

### Electromagnetic Model
- Coupled armature and field circuits
- Magnetic saturation effects
- Back EMF proportional to flux and speed
- Electromagnetic torque generation

### Thermal Model
- Heat generation from all loss sources
- Thermal dynamics with resistance and capacitance
- Temperature-dependent derating
- Cooling time constant analysis

### Mechanical Model
- Rotational dynamics with inertia
- Friction and windage losses
- Shaft torque analysis
- Speed response to load changes

### Electrical Model
- Differential equations for armature and field currents
- Voltage balance equations
- RMS value calculations
- Power flow analysis

## Installation

### Requirements
```bash
pip install numpy matplotlib scipy
```

### Running the Application
```bash
python3 shunt_motor_speed_control_advanced.py
```

## Usage Guide

### Quick Start
1. Launch the application
2. Go to "Main Control" tab
3. Click "Solve Example 30.25" to see the solution
4. Adjust motor parameters using sliders

### Running Simulations
1. Switch to "Dynamic Simulation" tab
2. Set desired series resistance and load torque
3. Select ODE solver (RK45 or Euler)
4. Click "Start Simulation"
5. Observe real-time graphs
6. Click "Stop" to pause, "Reset" to restart

### Analyzing Performance
1. **Loss Analysis Tab**: View detailed breakdown of losses
2. **Economic Analysis Tab**: Monitor operating costs
3. **Thermal & Derating Tab**: Check temperature and safety margins

## Advanced Features

### Auto-Scaling
- Window automatically adjusts when resized
- All graphs scale proportionally
- Responsive grid layout

### Method of Speed Control
The simulator demonstrates **armature resistance control**:
- Adding series resistance reduces speed at constant torque
- Inefficient method due to high losses
- Used for temporary speed reduction

### Advanced Controls
- Real-time parameter adjustment
- Multiple solver options
- Simultaneous multi-parameter monitoring

### Practical Applications
- Motor speed control design
- Loss analysis and optimization
- Economic feasibility studies
- Thermal management
- Educational tool for electrical engineering

## Technical Details

### State Variables
The simulator solves for 5 state variables:
1. Armature current (Ia)
2. Field current (If)
3. Angular velocity (ω)
4. Rotor angle (θ)
5. Motor temperature (T)

### Differential Equations

**Electrical:**
- dIa/dt = (V - Eb - Ia×(Ra + R_added)) / La
- dIf/dt = (V - If×Rf) / Lf

**Mechanical:**
- dω/dt = (T_em - T_load - B×ω) / J

**Thermal:**
- dT/dt = (P_losses - (T - T_amb)/R_th) / C_th

### Loss Models

**Copper Losses:**
- P_cu = Ia²×(Ra + R_added) + If²×Rf

**Iron Losses:**
- Hysteresis: P_h = k_h × f × B²
- Eddy Current: P_e = k_e × f² × B²

**Mechanical Losses:**
- Friction: P_f = B × ω²
- Windage: P_w = k_w × ω³

## Key Concepts

### For DC Shunt Motor with Constant Flux:
1. Back EMF (Eb) is proportional to speed (N)
2. Torque (T) is proportional to armature current (Ia)
3. Voltage equation: V = Eb + Ia × (Ra + R_added)
4. Adding series resistance reduces speed for same torque
5. Reducing load torque increases speed

## Educational Value

This simulator is ideal for:
- Understanding DC motor speed control methods
- Analyzing multi-physics interactions
- Learning about loss mechanisms
- Economic analysis of motor operation
- Thermal management principles
- Numerical ODE solving techniques

## Future Enhancements

Potential additions:
- Field flux control
- Ward-Leonard speed control
- Chopper-based control
- Four-quadrant operation
- Multiple motor comparison
- Export data to CSV
- Custom loss model input

## Author Notes

This comprehensive simulator combines:
- Accurate mathematical modeling
- Intuitive graphical interface
- Real-time simulation capabilities
- Practical engineering analysis
- Educational clarity

Perfect for students, engineers, and researchers studying DC motor control systems.

## License

Educational and research use encouraged.
