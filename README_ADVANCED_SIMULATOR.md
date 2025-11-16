# Advanced DC Motor Multi-Physics Simulator

## Overview

This comprehensive DC motor simulator provides advanced multi-physics analysis including electromagnetic, thermal, and mechanical modeling with real-time visualization and economic analysis.

## Features

### 1. **Problem Solving (Tab 1)**
   - **Problem 10**: 230V DC shunt motor with field resistance change
   - **Problem 11**: 250V DC shunt motor with flux weakening
   - Detailed step-by-step solutions with analysis

### 2. **Dynamic Simulation (Tab 2)**
   - Real-time ODE solvers:
     - **RK45**: 4th-5th order Runge-Kutta method (high accuracy)
     - **Euler**: Forward Euler method (fast computation)
   - Adjustable time step
   - Motor parameter sliders for real-time adjustment
   - Start/Stop/Reset controls

### 3. **Advanced Controls (Tab 3)**
   - **PWM Control**: Duty cycle and frequency adjustment
   - **PID Speed Control**: Tunable Kp, Ki, Kd parameters
   - **Real-time Power Monitoring**:
     - Input power
     - Output power
     - Detailed loss breakdown (copper, iron, mechanical, stray)
     - Efficiency calculation

### 4. **Thermal Analysis (Tab 4)**
   - Coupled thermal-electrical model
   - Temperature-dependent resistance
   - Thermal derating calculations
   - Insulation class analysis (B, F, H)
   - Lifetime estimation based on operating temperature

### 5. **Economic Analysis (Tab 5)**
   - Operating cost calculations
   - Lifecycle cost analysis
   - Energy consumption tracking
   - Efficiency improvement scenarios
   - VFD (Variable Frequency Drive) ROI analysis
   - CO2 emissions estimation

### 6. **Visualization (Tab 6)**
   - 9 real-time dynamic graphs:
     1. Armature current vs time
     2. Field current vs time
     3. Speed vs time
     4. Electromagnetic torque vs time
     5. Output power vs time
     6. Efficiency vs time
     7. Armature temperature vs time
     8. Field temperature vs time
     9. Torque-speed characteristic

## Multi-Physics Simulation

### Electromagnetic Model
- Coupled electrical equations for armature and field circuits
- Back EMF calculation: `Eb = Kb × If × ω`
- Electromagnetic torque: `T = Kt × If × Ia`
- Iron losses: Proportional to speed² and flux

### Thermal Model
- Heat generation from:
  - Armature copper losses: `I²Ra`
  - Field copper losses: `I²Rf`
  - Iron losses
  - Mechanical friction losses
- Temperature-dependent resistance: `R(T) = R₀(1 + α(T - T₀))`
- Thermal capacitance and resistance network
- Heat transfer to ambient

### Mechanical Model
- Torque balance: `J(dω/dt) = Tem - Tload - Bω - Tiron_loss`
- Moment of inertia effects
- Viscous friction
- Load torque application

## Installation

### Requirements
```bash
pip install numpy scipy matplotlib
```

### Python Dependencies
- Python 3.6+
- tkinter (usually included with Python)
- numpy
- scipy
- matplotlib

## Usage

### Running the Simulator
```bash
python3 advanced_dc_motor_simulator.py
```

### Solving Problems 10 and 11

1. Navigate to the **"Problems 10 & 11"** tab
2. Click **"Solve Problem 10"** to see the detailed solution for the 230V motor
3. Click **"Solve Problem 11"** to see the detailed solution for the 250V motor

### Running Dynamic Simulation

1. Go to **"Dynamic Simulation"** tab
2. Adjust motor parameters using sliders or text entry
3. Select ODE solver (RK45 recommended for accuracy)
4. Click **"Start"** to begin simulation
5. Use **"Stop"** to pause
6. Use **"Reset"** to clear and start over

### Advanced Controls

1. Navigate to **"Advanced Controls"** tab
2. Adjust PWM duty cycle and frequency
3. Set target speed for closed-loop control
4. Tune PID parameters (Kp, Ki, Kd)
5. Monitor real-time power consumption and efficiency

### Thermal Analysis

1. Go to **"Thermal Analysis"** tab
2. Set thermal parameters and ambient temperature
3. Click **"Calculate Derating"** for detailed thermal analysis
4. Review temperature margins and lifetime impact

### Economic Analysis

1. Navigate to **"Economic Analysis"** tab
2. Enter electricity cost, maintenance cost, and motor cost
3. Set operating hours per day
4. Click **"Calculate Economics"** for comprehensive cost analysis

## Problem Solutions

### Problem 10: 230V DC Shunt Motor

**Given:**
- Supply Voltage: 230 V
- Armature Resistance: 0.5 Ω
- Initial Field Resistance: 76 2/3 Ω (76.67 Ω)
- Additional Field Resistance: 38 1/3 Ω (38.33 Ω)
- No-load Current: 13 A at 1000 rpm
- Load Current: 42 A

**Solution:**
The new speed is **1400 rpm**

**Key Steps:**
1. Calculate field currents: If1 = 3.0 A, If2 = 2.0 A
2. Calculate armature currents: Ia1 = 10.0 A, Ia2 = 40.0 A
3. Calculate back EMFs: Eb1 = 225 V, Eb2 = 210 V
4. Apply speed equation: N2 = N1 × (Eb2/Eb1) × (If1/If2) = 1400 rpm

### Problem 11: 250V DC Shunt Motor with Flux Weakening

**Given:**
- Supply Voltage: 250 V
- Armature Resistance: 0.4 Ω
- Initial Speed: 1000 rpm at Ia = 25 A
- New Armature Current: 50 A
- Flux Reduction: 3%

**Solution:**
The new speed is **approximately 1000 rpm** (exact: depends on calculation)

**Key Steps:**
1. Calculate back EMFs: Eb1 = 240 V, Eb2 = 230 V
2. Apply flux ratio: Φ2/Φ1 = 0.97
3. Calculate new speed: N2 = N1 × (Eb2/Eb1) / (Φ2/Φ1)

## Technical Details

### Loss Breakdown

1. **Copper Losses**:
   - Armature: `Pcu_a = Ia² × Ra`
   - Field: `Pcu_f = If² × Rf`

2. **Iron Losses**:
   - Hysteresis losses
   - Eddy current losses
   - Approximated as: `Piron ∝ ω² × Φ`

3. **Mechanical Losses**:
   - Friction: `Pmech = B × ω²`
   - Windage losses

4. **Stray Losses**:
   - Approximately 0.5-1% of input power

### Efficiency Calculation
```
η = Pout / Pin × 100%
where:
Pin = V × (Ia + If)
Pout = T × ω
```

### Thermal Time Constants
- Armature: τa = Cth_a × Rth_a
- Field: τf = Cth_f × Rth_f

Typical values: 10-30 minutes for heating, longer for cooling

## Auto-Scaling Features

- Window resize automatically adjusts all GUI elements
- Graphs auto-scale to fit data
- Responsive layout using tkinter's pack and grid managers
- Canvas updates on window configure events

## Advanced Features

### 1. Temperature-Dependent Resistance
All resistance values automatically adjust based on winding temperature using the copper temperature coefficient (0.4%/°C).

### 2. Real-Time ODE Integration
Choose between:
- **RK45**: Adaptive step-size for high accuracy
- **Euler**: Fixed step-size for fast computation

### 3. Multi-Physics Coupling
Simultaneous solution of:
- Electrical equations (voltage, current)
- Mechanical equations (torque, speed)
- Thermal equations (temperature rise)

### 4. Economic Optimization
Analyze:
- Operating costs
- Energy consumption
- Efficiency improvements
- VFD installation ROI

## Practical Applications

This simulator is designed for:
- **Educational purposes**: Understanding DC motor behavior
- **Design verification**: Testing motor specifications
- **Energy audits**: Analyzing power consumption
- **Maintenance planning**: Thermal derating analysis
- **Economic justification**: Cost-benefit analysis
- **Research**: Multi-physics modeling

## Tips for Best Results

1. **Start with default parameters** to understand basic behavior
2. **Use RK45 solver** for accurate transient analysis
3. **Monitor temperature** to avoid overheating
4. **Run economic analysis** to optimize operating conditions
5. **Compare different scenarios** by adjusting parameters
6. **Export results** by taking screenshots of graphs

## Troubleshooting

- **Simulation won't start**: Check that all parameters are positive values
- **Unstable results**: Reduce time step or use RK45 solver
- **Temperature too high**: Reduce load torque or increase cooling
- **Low efficiency**: Check for excessive losses in controls tab

## Author
Created as part of advanced electrical engineering coursework for DC motor analysis and design.

## License
Educational use only.
