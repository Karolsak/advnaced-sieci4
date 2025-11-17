# Installation and Setup Guide

## Advanced Transformer Load Distribution Simulator

### System Requirements

- **Python**: 3.7 or higher
- **Operating System**: Linux, Windows, macOS
- **RAM**: Minimum 2GB
- **Display**: Minimum resolution 1280x720 (recommended: 1400x900 or higher)

### Required Python Packages

The application requires the following Python packages:

```bash
numpy>=1.19.0
scipy>=1.5.0
matplotlib>=3.3.0
```

Note: `tkinter` is usually included with Python by default. If not, see platform-specific installation below.

### Installation Instructions

#### Option 1: Using pip (Recommended)

1. **Install required packages:**
   ```bash
   pip3 install numpy scipy matplotlib
   ```

2. **Verify tkinter installation:**
   ```bash
   python3 -c "import tkinter; print('Tkinter is installed')"
   ```

   If you get an error, install tkinter:

   **Ubuntu/Debian:**
   ```bash
   sudo apt-get update
   sudo apt-get install python3-tk
   ```

   **Fedora/RHEL:**
   ```bash
   sudo dnf install python3-tkinter
   ```

   **macOS:**
   ```bash
   # Tkinter comes with Python installation
   # If missing, reinstall Python from python.org
   brew install python-tk
   ```

   **Windows:**
   ```bash
   # Tkinter is included with Python installer
   # Make sure to check "tcl/tk and IDLE" during installation
   ```

3. **Run the application:**
   ```bash
   python3 transformer_load_distribution_advanced.py
   ```

#### Option 2: Using a Virtual Environment (Best Practice)

1. **Create a virtual environment:**
   ```bash
   python3 -m venv transformer_env
   ```

2. **Activate the virtual environment:**

   **Linux/macOS:**
   ```bash
   source transformer_env/bin/activate
   ```

   **Windows:**
   ```bash
   transformer_env\Scripts\activate
   ```

3. **Install packages:**
   ```bash
   pip install numpy scipy matplotlib
   ```

4. **Run the application:**
   ```bash
   python transformer_load_distribution_advanced.py
   ```

5. **Deactivate when done:**
   ```bash
   deactivate
   ```

#### Option 3: Using Anaconda/Miniconda

1. **Create conda environment:**
   ```bash
   conda create -n transformer python=3.9
   ```

2. **Activate environment:**
   ```bash
   conda activate transformer
   ```

3. **Install packages:**
   ```bash
   conda install numpy scipy matplotlib
   ```

4. **Run the application:**
   ```bash
   python transformer_load_distribution_advanced.py
   ```

### Testing the Installation

#### 1. Quick Syntax Check
```bash
python3 -m py_compile transformer_load_distribution_advanced.py
```

If no errors appear, the syntax is correct.

#### 2. Run Simple Test (No GUI dependencies)
```bash
python3 transformer_simple_test.py
```

This will run the core calculations and display results in the terminal.

#### 3. Run Full GUI Application
```bash
python3 transformer_load_distribution_advanced.py
```

### Troubleshooting

#### Problem: "ModuleNotFoundError: No module named 'tkinter'"

**Solution:**
- Linux: `sudo apt-get install python3-tk`
- macOS: `brew install python-tk`
- Windows: Reinstall Python with tcl/tk support

#### Problem: "ModuleNotFoundError: No module named 'numpy'"

**Solution:**
```bash
pip3 install numpy scipy matplotlib
```

#### Problem: GUI appears but graphs don't show

**Solution:**
- Update matplotlib: `pip3 install --upgrade matplotlib`
- Check backend: Run `python3 -c "import matplotlib; print(matplotlib.get_backend())"`
- If using SSH, enable X11 forwarding: `ssh -X user@host`

#### Problem: Application runs slowly

**Solutions:**
1. Use Euler method instead of RK45 for dynamic simulation
2. Reduce simulation duration
3. Close other applications to free up RAM
4. Update to latest numpy/scipy versions

#### Problem: Window doesn't resize properly

**Solution:**
- This is a known issue with some window managers
- Try maximizing the window manually
- The auto-scaling should activate after a few seconds

### File Structure

After installation, you should have these files:

```
advnaced-sieci4/
├── transformer_load_distribution_advanced.py  # Main GUI application
├── transformer_core.py                        # Core calculation module
├── transformer_simple_test.py                 # Simple test (no dependencies)
├── test_transformer_calculations.py           # Full test (requires packages)
├── README_transformer_advanced.md             # User guide
├── INSTALLATION.md                            # This file
└── results/                                   # Created automatically for saved results
```

### Running Without GUI

If you only need calculations without the GUI interface, use the core module:

```python
from transformer_core import TransformerLoadDistribution

# Create instance
transformer = TransformerLoadDistribution()

# Set parameters
transformer.S_A_rated = 2000  # kVA
transformer.S_B_rated = 4000  # kVA
transformer.S_load = 5000     # kVA
transformer.pf = 0.8          # Power factor

# Calculate
results = transformer.calculate_load_distribution()

# Print results
print(f"Transformer A: {results['S_A']:.2f} kVA")
print(f"Transformer B: {results['S_B']:.2f} kVA")
```

### Performance Optimization

For better performance:

1. **Use compiled NumPy:**
   ```bash
   pip install numpy --no-binary numpy
   ```

2. **Install Intel MKL (if on Intel processor):**
   ```bash
   pip install mkl
   ```

3. **For large-scale simulations, consider:**
   - Using PyPy instead of CPython
   - Compiling with Cython
   - Using numba for JIT compilation

### Updating

To update the application:

1. **Pull latest changes:**
   ```bash
   git pull origin main
   ```

2. **Update dependencies:**
   ```bash
   pip3 install --upgrade numpy scipy matplotlib
   ```

### Uninstallation

1. **Remove virtual environment (if used):**
   ```bash
   rm -rf transformer_env
   ```

2. **Remove packages (global installation):**
   ```bash
   pip3 uninstall numpy scipy matplotlib
   ```

3. **Remove application files:**
   ```bash
   rm transformer_load_distribution_advanced.py
   rm transformer_core.py
   rm transformer_simple_test.py
   ```

### Support

For issues, questions, or contributions:
- Check the README_transformer_advanced.md for usage documentation
- Run the simple test to verify calculations
- Ensure all dependencies are correctly installed

### Version Information

- **Application Version**: 1.0
- **Python Compatibility**: 3.7+
- **Last Updated**: 2025
- **Platform**: Cross-platform (Linux, Windows, macOS)

### License

Educational and research use.

---

**Quick Start Command (Linux/macOS):**
```bash
pip3 install numpy scipy matplotlib && python3 transformer_load_distribution_advanced.py
```

**Quick Start Command (Windows):**
```bash
pip install numpy scipy matplotlib && python transformer_load_distribution_advanced.py
```
