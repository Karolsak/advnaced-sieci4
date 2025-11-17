# Advanced Salient-Pole Synchronous Generator Simulator
## Example 7.3 - Multi-Physics Interactive Lab

[![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-orange.svg)](https://jupyter.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 📋 Overview

This advanced interactive simulator solves **Example 7.3** from electrical machines theory and provides a comprehensive multi-physics simulation environment for salient-pole synchronous generators.

### Problem Statement

A salient-pole synchronous generator with the following specifications:
- **Apparent power**: Sn = 50 kVA
- **Line voltage**: V1Ln = 380 V
- **Frequency**: fn = 60 Hz
- **Speed**: nn = 1800 rpm
- **Power factor**: cos φn = 0.82

**Laboratory test results:**
1. **Slip test** (n = 1768 rpm, Vs = 115 V, fs = 60 Hz):
   - Imax = 22.2 A
   - Imin = 11.3 A
2. **Field winding**: Rf = 0.8 Ω (copper)
3. **No-load excitation**: If0 = 10.5 A

**Find:**
- (a) Synchronous reactances Xsd and Xsq
- (b) Nominal field excitation current Ifn
- (c) Field winding voltage at 120°C

---

## ✨ Features

### 🔬 Theoretical Solution
- ✅ Complete analytical solution to Example 7.3
- ✅ Step-by-step calculations with detailed explanations
- ✅ D-Q axis transformation and analysis
- ✅ Temperature-corrected resistance calculations

### ⚡ Multi-Physics Simulation
- **Electromagnetic Model**:
  - D-Q axis representation
  - Salient-pole torque equations
  - Linear magnetization curve
  - RMS voltage and current calculations

- **Thermal Model**:
  - Coupled stator and rotor thermal dynamics
  - Heat transfer equations
  - Temperature-dependent resistance
  - Thermal capacitance and resistance modeling

- **Mechanical Model**:
  - Shaft dynamics (moment of inertia, damping)
  - Torque balance equations
  - Speed control and transient response

### 🎯 Dynamic Simulation
- **ODE Solvers**:
  - RK45 (Adaptive Runge-Kutta, 4th/5th order)
  - Euler (Forward Euler method)
  - Configurable time step
  - Real-time state variable tracking

### 📊 Comprehensive Analysis
- **Loss Breakdown**:
  - Copper losses (stator and rotor)
  - Iron/core losses
  - Mechanical friction losses
  - Stray load losses
  - Efficiency calculations

- **Economic Analysis**:
  - Annual energy cost
  - Levelized Cost of Energy (LCOE)
  - Present value calculations
  - Payback period analysis
  - Cost breakdown by component

- **Thermal Derating**:
  - Temperature monitoring
  - Class F insulation limits (155°C)
  - Thermal margin calculation
  - Safety warnings

### 🎨 Interactive GUI
- **User Controls**:
  - Real-time parameter adjustment via sliders
  - Multiple ODE solver selection
  - Start/Stop/Reset buttons
  - Tabbed interface for organized viewing

- **Visualization**:
  - Auto-scaling dynamic plots
  - 6 simultaneous plot views
  - Real-time data updates
  - Professional formatting

---

## 🚀 Installation

### Prerequisites

```bash
pip install numpy scipy matplotlib ipywidgets
```

### Optional (for Jupyter Notebook)

```bash
pip install jupyter
jupyter nbextension enable --py widgetsnbextension
```

---

## 💻 Usage

### Option 1: Python Script

Run the complete simulator:

```bash
python example_7_3_advanced_simulator.py
```

This will:
1. Display the theoretical solution to Example 7.3
2. Launch the interactive GUI with ipywidgets

### Option 2: Jupyter Notebook (Recommended)

For the best interactive experience:

```bash
jupyter notebook example_7_3_interactive_lab.ipynb
```

Then run all cells to:
1. Load all modules
2. Display theoretical solution
3. Initialize the multi-physics engine
4. Launch the interactive simulator

### Option 3: Quick Test

To run a quick verification:

```python
from example_7_3_advanced_simulator import Example73Solution, SynchronousGeneratorMultiPhysics

# Solve theoretical problem
solution = Example73Solution()
solution.print_results()

# Run quick simulation
generator = SynchronousGeneratorMultiPhysics(solution)
t_eval, y_sol = generator.simulate_dynamics(
    t_span=(0, 5),
    Tm_profile=200,  # Nm
    If_profile=solution.Ifn,
    load_pf=0.82,
    method='RK45'
)

print(f"Simulation completed: {len(t_eval)} time steps")
print(f"Final efficiency: {generator.history['efficiency'][-1]:.2f}%")
```

---

## 📈 Results

### Theoretical Solution (Example 7.3)

```
================================================================================
EXAMPLE 7.3 - SALIENT-POLE SYNCHRONOUS GENERATOR SOLUTION
================================================================================

(a) SYNCHRONOUS REACTANCES:
    Xsd (Direct-axis reactance)     = 10.1770 Ω
    Xsq (Quadrature-axis reactance) = 5.1802 Ω

(b) NOMINAL FIELD EXCITATION CURRENT:
    Ifn = 14.0886 A

(c) FIELD WINDING VOLTAGE AT 120°C:
    Rf at 120°C = 1.1747 Ω
    Vf at 120°C = 16.5470 V

ADDITIONAL RESULTS:
    Phase voltage (Vph)        = 219.39 V
    Nominal current (In)       = 75.99 A
    Excitation EMF (E0)        = 294.26 V
    Direct-axis current (Id)   = 43.47 A
    Quadrature-axis current (Iq) = 62.31 A
    Power factor angle         = 34.92°
================================================================================
```

### Sample Simulation Output

After running a 5-second simulation with nominal parameters:

- **Final stator temperature**: ~35°C
- **Final rotor temperature**: ~32°C
- **Final speed**: 1800 rpm (stable)
- **Electromagnetic torque**: ~200 Nm
- **Output power**: ~37.7 kW
- **Efficiency**: ~94.5%

**Loss Distribution**:
- Copper losses: ~1500 W (65%)
- Iron losses: ~500 W (22%)
- Friction losses: ~200 W (9%)
- Stray losses: ~100 W (4%)

---

## 🎛️ Control Parameters

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| Power Factor | 0.5 - 1.0 | 0.82 | Load power factor (cos φ) |
| Load Current | 0 - 114 A | 76 A | Stator load current |
| Field Current | 0 - 20 A | 14.1 A | Field excitation current |
| Speed | 0 - 2400 rpm | 1800 rpm | Mechanical shaft speed |
| Mech. Torque | 0 - 500 Nm | 200 Nm | Applied mechanical torque |
| Ambient Temp | 0 - 50 °C | 25 °C | Ambient temperature |
| Sim. Time | 0.1 - 20 s | 5 s | Total simulation duration |
| ODE Solver | RK45/Euler | RK45 | Integration method |

---

## 📊 GUI Tabs

### ⚙️ Controls Tab
- All parameter sliders
- Solver selection
- Control buttons (Start/Stop/Reset)

### 📈 Results Tab
- Real-time simulation output
- Summary statistics
- 6 dynamic plots:
  1. D-Q axis currents
  2. Electromagnetic & mechanical torque
  3. Rotor speed
  4. Stator and rotor temperature
  5. Output power & efficiency
  6. Excitation EMF & field current

### 💰 Economics Tab
- Annual energy costs
- LCOE calculations
- Present value analysis
- Cost breakdown

### ⚡ Losses Tab
- Detailed loss breakdown
- Power flow diagram
- Loss distribution chart
- Thermal derating analysis

---

## 🔧 Technical Details

### Mathematical Models

#### Electromagnetic Equations (D-Q Frame)

**Voltage equations:**
```
Vd = -Ra·Id - ωs·Lsq·Iq
Vq = -Ra·Iq + ωs·Lsd·Id + E0
```

**Torque equation:**
```
Te = (3p/2) · [E0·Iq/ωs + (Lsd - Lsq)·Id·Iq]
```

#### Thermal Equations

**Stator:**
```
dTstator/dt = (Pstator - (Tstator - Tamb)/Rth_stator) / Cth_stator
```

**Rotor:**
```
dTrotor/dt = (Protor - (Trotor - Tamb)/Rth_rotor) / Cth_rotor
```

#### Mechanical Equation

```
J·dω/dt = Te - Tm - B·ω
```

Where:
- `J` = moment of inertia (0.5 kg·m²)
- `B` = damping coefficient (0.01 N·m·s)
- `Te` = electromagnetic torque
- `Tm` = mechanical load torque
- `ω` = angular velocity

### Loss Calculations

**Copper losses:**
```
Pcopper = 3·Ra·(Id² + Iq²) + Rf·If²
```

**Iron losses:**
```
Piron = k_iron · (f/fn)² · Sn
```

**Friction losses:**
```
Pfriction = k_friction · ω²
```

**Stray losses:**
```
Pstray = k_stray · (Id² + Iq²)
```

---

## 📚 Educational Applications

This simulator is ideal for:

1. **Electrical Machines Courses**
   - Understanding salient-pole generators
   - D-Q axis transformation
   - Synchronous machine operation

2. **Power Systems Labs**
   - Generator dynamics
   - Load flow analysis
   - Stability studies

3. **Control Systems**
   - Speed control
   - Voltage regulation
   - Excitation control

4. **Thermal Management**
   - Heat dissipation
   - Temperature rise
   - Derating calculations

5. **Economic Analysis**
   - Life-cycle costing
   - Energy efficiency
   - ROI calculations

---

## 🔬 Advanced Features

### Multi-Physics Coupling

The simulator implements **bi-directional coupling** between physical domains:

```
Electromagnetic ←→ Thermal ←→ Mechanical
      ↓                ↓            ↓
  Losses → Heat → Temperature → Resistance
      ↓                              ↓
  Torque ←← Speed ←← Load ←← Efficiency
```

### Numerical Methods

**RK45 (Runge-Kutta-Fehlberg)**:
- Adaptive step size
- Error control
- Higher accuracy
- Recommended for most simulations

**Euler Method**:
- Fixed step size
- Simple implementation
- Faster execution
- Good for comparison studies

---

## 🎯 Practical Applications

1. **Generator Design**
   - Parameter optimization
   - Performance prediction
   - Thermal limits determination

2. **Commissioning**
   - Test result validation
   - Performance verification
   - Thermal testing

3. **Maintenance**
   - Condition monitoring
   - Efficiency degradation
   - Thermal aging

4. **Education**
   - Hands-on learning
   - Virtual laboratory
   - Experiment design

---

## 📖 References

1. **Electrical Machines Theory**: Salient-pole synchronous generators
2. **Slip Test Method**: For determining Xsd and Xsq
3. **D-Q Transformation**: Park's transformation for AC machines
4. **Thermal Modeling**: Heat transfer in electrical machines
5. **Economic Analysis**: LCOE and life-cycle costing methods

---

## 🐛 Troubleshooting

### Common Issues

**1. Widgets not displaying in Jupyter:**
```bash
jupyter nbextension enable --py widgetsnbextension
jupyter notebook
```

**2. Import errors:**
```bash
pip install --upgrade numpy scipy matplotlib ipywidgets
```

**3. Simulation too slow:**
- Use RK45 with larger max_step
- Reduce simulation time
- Decrease number of plot points

**4. Numerical instability:**
- Reduce time step (dt)
- Check parameter ranges
- Use RK45 instead of Euler

---

## 🤝 Contributing

Contributions are welcome! Areas for enhancement:

- [ ] Saturation effects (non-linear magnetization)
- [ ] Damper winding modeling
- [ ] Transient stability analysis
- [ ] Harmonic analysis
- [ ] 3D thermal modeling
- [ ] Real-time hardware-in-the-loop

---

## 📄 License

MIT License - Feel free to use for education and research.

---

## 👨‍💻 Author

Developed for advanced electrical engineering education with practical applications in power systems and electrical machines.

---

## 🎓 Citation

If you use this simulator in your research or teaching, please cite:

```bibtex
@software{sync_gen_simulator_2025,
  title={Advanced Salient-Pole Synchronous Generator Simulator},
  author={Example 7.3 Solution},
  year={2025},
  description={Multi-physics interactive simulation tool for electrical machines education}
}
```

---

## 📞 Support

For questions, issues, or suggestions:
- Create an issue in the repository
- Check the troubleshooting section
- Review the example notebooks

---

**Made with ⚡ for Electrical Engineering Education**

Last updated: 2025
