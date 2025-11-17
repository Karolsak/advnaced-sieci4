# Quick Start Guide - Transformer Load Distribution Simulator

## Problem Statement

**Calculate how two parallel transformers share a 5,000 kVA load:**

- **Transformer A**: 2,000 kVA, R=2%, X=8%
- **Transformer B**: 4,000 kVA, R=1.6%, X=3%
- **Load**: 5,000 kVA at 0.8 power factor lagging

## Quick Answer

Running the simulation shows:

```
Transformer A supplies: ~858 kVA (42.9% loading)
Transformer B supplies: ~4,163 kVA (104.1% loading) ⚠️ OVERLOADED
```

**Important**: Transformer B is operating above its rated capacity and will overheat!

## 30-Second Setup

```bash
# Install dependencies
pip3 install numpy scipy matplotlib

# Run GUI application
python3 transformer_load_distribution_advanced.py
```

**OR** run simple test without GUI:

```bash
python3 transformer_simple_test.py
```

## Using the GUI - 5 Steps

### 1. Launch Application
```bash
python3 transformer_load_distribution_advanced.py
```

### 2. Set Parameters (Input Parameters Tab)
- Adjust sliders or keep default values
- Default values match the problem statement

### 3. Click START Button
- Located at bottom of window
- Results appear immediately

### 4. View Results (Multiple Tabs)

**Load Distribution Tab:**
- Bar charts showing load sharing
- Current distribution
- Loading percentages

**Loss Breakdown Tab:**
- Pie charts for each transformer
- Copper, iron, stray, and mechanical losses
- Total loss comparison

**Dynamic Simulation Tab:**
- Select ODE solver (RK45 or Euler)
- Set duration (seconds)
- Click "Run Simulation"
- Watch temperature evolution over time

**Thermal Analysis Tab:**
- Temperature profiles
- Thermal time constants
- Derating recommendations

**Economic Analysis Tab:**
- Enter cost parameters
- Click "Calculate Economics"
- View ROI and payback period

### 5. Save Results
- Menu → File → Save Results
- Exports to timestamped text file

## Key Features At-a-Glance

| Feature | Location | Purpose |
|---------|----------|---------|
| **Interactive Sliders** | Input Parameters tab | Adjust transformer ratings, impedances, load |
| **START/STOP/RESET** | Bottom control panel | Control simulation |
| **ODE Solvers** | Dynamic Simulation tab | Choose RK45 (accurate) or Euler (fast) |
| **Loss Analysis** | Loss Breakdown tab | Detailed loss categorization |
| **Temperature** | Thermal Analysis tab | Thermal performance and derating |
| **Economics** | Economic Analysis tab | Cost analysis and ROI |
| **Auto-scale** | All graphs | Automatically resize with window |

## Understanding the Results

### Load Distribution Principle

**Key Formula**: Load sharing is **inversely proportional** to impedance

```
S_A / S_B = |Z_B| / |Z_A|
```

- **Lower impedance** → **Higher load**
- Transformer B has lower impedance (0.042 pu vs 0.206 pu)
- Therefore, Transformer B carries more load
- In this case: **TOO MUCH** load (104% vs rated)

### Why Transformer B is Overloaded

1. **Impedance Comparison:**
   - Z_A = 0.206 pu (higher)
   - Z_B = 0.042 pu (lower)
   - Ratio: 4.85:1

2. **Load Distribution:**
   - B carries 4.85× more load than A
   - 858 kVA vs 4,163 kVA

3. **Problem:**
   - B's rating: 4,000 kVA
   - B's actual load: 4,163 kVA
   - Overload: 4.1%

### What the Warnings Mean

⚠️ **Temperature exceeds 85°C**: Transformer will overheat
- **Solution**: Reduce total load or add cooling

⚠️ **Loading > 100%**: Operating above rated capacity
- **Solution**: Redistribute load or add transformer capacity

✓ **Within thermal limits**: Safe operation
- Continue normal operation

## Common Use Cases

### 1. Design Verification
**Question**: Will these two transformers work in parallel for this load?
**Answer**: Run START button → Check loading percentages
- Both < 100%: ✓ Yes
- Either > 100%: ✗ No, resize or add capacity

### 2. Temperature Prediction
**Question**: How hot will the transformers get?
**Answer**: Run Dynamic Simulation → View temperature curves
- Check steady-state temperature (final value)
- Verify < 85°C for safe operation

### 3. Economic Comparison
**Question**: What's the annual operating cost?
**Answer**: Economic Analysis tab → Enter costs → Calculate
- Shows energy loss costs
- Calculates ROI and payback

### 4. What-If Analysis
**Question**: What if I change the load power factor to 0.95?
**Answer**:
1. Input Parameters tab → Adjust power factor slider
2. Click START
3. Compare new results with previous

## Tips & Tricks

### Faster Simulation
- Use **Euler** method instead of RK45
- Reduce simulation duration
- Close unused tabs

### Better Accuracy
- Use **RK45** solver
- Increase simulation duration
- Reduce time steps

### Understanding Graphs

**Temperature Evolution** (Dynamic Simulation):
- **Rising curve**: Heating up
- **Flat line**: Reached steady-state
- **Higher curves**: More heavily loaded transformer

**Loss Pie Charts** (Loss Breakdown):
- **Copper** (red): Increases with load²
- **Iron** (blue): Constant regardless of load
- **Stray** (cyan): Varies with load^1.8
- **Mechanical** (green): Constant

**Loading Bars** (Load Distribution):
- **< 100%**: Safe (green zone)
- **> 100%**: Overload (red zone)
- **Aim for**: 80-90% for optimal efficiency

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| **Ctrl+R** | Run simulation (if implemented) |
| **Ctrl+S** | Save results |
| **Ctrl+Q** | Quit application |
| **F1** | Help |

## Interpreting the Math

### Impedance on Common Base

Original impedances are on different bases (2000 kVA vs 4000 kVA).
Must convert to common base:

```
Z_new = Z_old × (S_new / S_old)
```

Example for Transformer A:
```
Z_A = (0.02 + j0.08) × (5000 / 2000) = 0.05 + j0.20 pu
```

### Admittance Method

Current distribution uses admittance (Y = 1/Z):

```
I_A = I_total × (Y_A / Y_total)
I_B = I_total × (Y_B / Y_total)
```

### Power Calculation

With equal voltages:
```
S = √3 × V × I
```

## Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| Won't start | Install: `pip3 install numpy scipy matplotlib` |
| No graphs | Install: `sudo apt-get install python3-tk` |
| Slow | Use Euler method, reduce duration |
| Wrong results | Check input parameters, verify units |
| Crashed | Restart, check RAM usage |

## Example Output (Terminal)

```
TRANSFORMER PARALLEL LOAD DISTRIBUTION ANALYSIS
================================================================

INPUT PARAMETERS:
Transformer A: 2000 kVA, R=2%, X=8%
Transformer B: 4000 kVA, R=1.6%, X=3%
Total Load: 5000 kVA at 0.8 p.f. lagging

LOAD DISTRIBUTION RESULTS:
Transformer A supplies: 858.24 kVA (42.91% loading)
Transformer B supplies: 4163.05 kVA (104.08% loading)
Total supplied: 5021.29 kVA

⚠ WARNING: Transformer B exceeds rated capacity!
   Recommended action: Reduce load or add capacity
```

## Next Steps

1. **Experiment**: Try different load values with sliders
2. **Learn**: Read README_transformer_advanced.md for detailed theory
3. **Optimize**: Use Economic Analysis to minimize costs
4. **Design**: Verify thermal limits for your specific application
5. **Document**: Save results for your reports

## Support

- **Documentation**: README_transformer_advanced.md
- **Installation**: INSTALLATION.md
- **Test calculations**: `python3 transformer_simple_test.py`

---

**Remember**: This tool is for educational and design purposes. Always verify critical designs with industry standards and professional review.
