"""
Simple test script for transformer load distribution
Uses only standard Python library (no external dependencies)
"""

import math

def calculate_transformer_load_distribution():
    """
    Solve the transformer parallel load distribution problem

    Problem: 2,000-kVA transformer (A) in parallel with 4,000-kVA transformer (B)
    supplying 5,000 kVA at 0.8 p.f. lagging

    Transformer A: R=2%, X=8%
    Transformer B: R=1.6%, X=3%
    """

    print("="*80)
    print("TRANSFORMER PARALLEL LOAD DISTRIBUTION - ANALYTICAL SOLUTION")
    print("="*80)
    print()

    # Input parameters
    S_A_rated = 2000  # kVA
    R_A_percent = 2.0
    X_A_percent = 8.0

    S_B_rated = 4000  # kVA
    R_B_percent = 1.6
    X_B_percent = 3.0

    S_load = 5000  # kVA
    pf = 0.8
    voltage = 11.0  # kV (assumed)

    print("INPUT PARAMETERS:")
    print("-"*80)
    print(f"Transformer A: {S_A_rated} kVA, R = {R_A_percent}%, X = {X_A_percent}%")
    print(f"Transformer B: {S_B_rated} kVA, R = {R_B_percent}%, X = {X_B_percent}%")
    print(f"Total Load:    {S_load} kVA at {pf} p.f. lagging")
    print(f"Voltage:       {voltage} kV")
    print()

    # Step 1: Convert impedances to common base (5000 kVA)
    base_S = S_load

    # Impedance in per-unit on own base
    Z_A_pu_own = complex(R_A_percent/100, X_A_percent/100)
    Z_B_pu_own = complex(R_B_percent/100, X_B_percent/100)

    # Convert to common base
    Z_A_pu = Z_A_pu_own * (base_S / S_A_rated)
    Z_B_pu = Z_B_pu_own * (base_S / S_B_rated)

    print("STEP 1: IMPEDANCE CALCULATION (Common Base = 5000 kVA)")
    print("-"*80)
    print(f"Z_A (own base):    {Z_A_pu_own.real:.4f} + j{Z_A_pu_own.imag:.4f} pu")
    print(f"Z_A (common base): {Z_A_pu.real:.4f} + j{Z_A_pu.imag:.4f} pu")
    print(f"|Z_A| = {abs(Z_A_pu):.6f} pu")
    print(f"∠Z_A = {math.degrees(math.atan2(Z_A_pu.imag, Z_A_pu.real)):.2f}°")
    print()
    print(f"Z_B (own base):    {Z_B_pu_own.real:.4f} + j{Z_B_pu_own.imag:.4f} pu")
    print(f"Z_B (common base): {Z_B_pu.real:.4f} + j{Z_B_pu.imag:.4f} pu")
    print(f"|Z_B| = {abs(Z_B_pu):.6f} pu")
    print(f"∠Z_B = {math.degrees(math.atan2(Z_B_pu.imag, Z_B_pu.real)):.2f}°")
    print()

    # Step 2: Calculate admittances
    Y_A = 1 / Z_A_pu
    Y_B = 1 / Z_B_pu
    Y_total = Y_A + Y_B

    print("STEP 2: ADMITTANCE CALCULATION")
    print("-"*80)
    print(f"Y_A = 1/Z_A = {Y_A.real:.6f} + j{Y_A.imag:.6f} pu")
    print(f"|Y_A| = {abs(Y_A):.6f} pu")
    print()
    print(f"Y_B = 1/Z_B = {Y_B.real:.6f} + j{Y_B.imag:.6f} pu")
    print(f"|Y_B| = {abs(Y_B):.6f} pu")
    print()
    print(f"Y_total = Y_A + Y_B = {Y_total.real:.6f} + j{Y_total.imag:.6f} pu")
    print(f"|Y_total| = {abs(Y_total):.6f} pu")
    print()

    # Step 3: Load distribution
    # For parallel transformers with equal terminal voltages:
    # I_A / I_total = Y_A / Y_total
    # S_A / S_total = I_A / I_total (for equal voltages and similar power factors)

    # Calculate total load current
    I_total = S_load * 1000 / (math.sqrt(3) * voltage * 1000)  # Amps

    # Current distribution
    I_A_complex = (Y_A / Y_total) * I_total
    I_B_complex = (Y_B / Y_total) * I_total

    I_A = abs(I_A_complex)
    I_B = abs(I_B_complex)

    # Power distribution (S = √3 * V * I)
    S_A = math.sqrt(3) * voltage * 1000 * I_A / 1000  # kVA
    S_B = math.sqrt(3) * voltage * 1000 * I_B / 1000  # kVA

    print("STEP 3: LOAD DISTRIBUTION")
    print("-"*80)
    print(f"Total load current: {I_total:.2f} A")
    print()
    print(f"Current ratio: I_A/I_total = |Y_A/Y_total| = {abs(Y_A/Y_total):.6f}")
    print(f"Current ratio: I_B/I_total = |Y_B/Y_total| = {abs(Y_B/Y_total):.6f}")
    print()
    print(f"Transformer A current: I_A = {I_A:.2f} A")
    print(f"Transformer B current: I_B = {I_B:.2f} A")
    print(f"Sum of currents: {I_A + I_B:.2f} A (should equal {I_total:.2f} A)")
    print()

    # Step 4: Calculate kVA supplied
    loading_A = (S_A / S_A_rated) * 100
    loading_B = (S_B / S_B_rated) * 100

    print("="*80)
    print("FINAL RESULTS - kVA SUPPLIED BY EACH TRANSFORMER")
    print("="*80)
    print()
    print(f"┌{'─'*76}┐")
    print(f"│ Transformer A supplies: {S_A:8.2f} kVA  ({loading_A:5.2f}% of rated capacity) │")
    print(f"│ Transformer B supplies: {S_B:8.2f} kVA  ({loading_B:5.2f}% of rated capacity) │")
    print(f"├{'─'*76}┤")
    print(f"│ Total supplied:         {S_A + S_B:8.2f} kVA  (Load demand: {S_load} kVA)      │")
    print(f"│ Error:                  {abs(S_A + S_B - S_load):8.2f} kVA                              │")
    print(f"└{'─'*76}┘")
    print()

    # Step 5: Additional analysis
    print("ADDITIONAL ANALYSIS:")
    print("-"*80)

    # Impedance ratio vs power ratio
    impedance_ratio = abs(Z_A_pu) / abs(Z_B_pu)
    power_ratio = S_B / S_A  # Note: inverse relationship

    print(f"Impedance ratio (|Z_A|/|Z_B|):        {impedance_ratio:.4f}")
    print(f"Power ratio (S_B/S_A):                {power_ratio:.4f}")
    print(f"Expected: S_B/S_A ≈ |Z_A|/|Z_B|       (inverse proportionality)")
    print(f"Verification: {abs(impedance_ratio - power_ratio) < 0.1} (within tolerance)")
    print()

    # Loss calculation
    print("LOSS ESTIMATION:")
    print("-"*80)

    # Copper losses (I²R losses)
    loading_A_ratio = S_A / S_A_rated
    loading_B_ratio = S_B / S_B_rated

    copper_loss_A = (loading_A_ratio ** 2) * (R_A_percent / 100) * S_A_rated
    copper_loss_B = (loading_B_ratio ** 2) * (R_B_percent / 100) * S_B_rated

    # Iron losses (approximately constant, assumed 0.2% of rated)
    iron_loss_A = 0.002 * S_A_rated
    iron_loss_B = 0.002 * S_B_rated

    total_loss_A = copper_loss_A + iron_loss_A
    total_loss_B = copper_loss_B + iron_loss_B
    total_losses = total_loss_A + total_loss_B

    print(f"Transformer A:")
    print(f"  Copper losses: {copper_loss_A:.2f} kW")
    print(f"  Iron losses:   {iron_loss_A:.2f} kW")
    print(f"  Total losses:  {total_loss_A:.2f} kW")
    print()
    print(f"Transformer B:")
    print(f"  Copper losses: {copper_loss_B:.2f} kW")
    print(f"  Iron losses:   {iron_loss_B:.2f} kW")
    print(f"  Total losses:  {total_loss_B:.2f} kW")
    print()
    print(f"Combined total losses: {total_losses:.2f} kW")
    print()

    # Efficiency
    P_out = S_load * pf  # kW
    P_in = P_out + total_losses
    efficiency = (P_out / P_in) * 100

    print(f"Overall System Efficiency:")
    print(f"  Output power:  {P_out:.2f} kW")
    print(f"  Input power:   {P_in:.2f} kW")
    print(f"  Efficiency:    {efficiency:.3f}%")
    print()

    # Temperature estimation (simplified)
    ambient_temp = 25  # °C
    delta_T_rated = 65  # °C rise at rated load

    temp_A = ambient_temp + delta_T_rated * (loading_A_ratio ** 2)
    temp_B = ambient_temp + delta_T_rated * (loading_B_ratio ** 2)

    print("THERMAL ANALYSIS (Steady-State Estimate):")
    print("-"*80)
    print(f"Ambient temperature:      {ambient_temp}°C")
    print(f"Transformer A temperature: {temp_A:.1f}°C (rise: {temp_A - ambient_temp:.1f}°C)")
    print(f"Transformer B temperature: {temp_B:.1f}°C (rise: {temp_B - ambient_temp:.1f}°C)")
    print()

    if temp_A > 85 or temp_B > 85:
        print("⚠ WARNING: Temperature exceeds typical maximum (85°C)")
        print("  Consider derating or improved cooling")
    else:
        print("✓ Temperatures within normal operating range")

    print()
    print("="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)

    return {
        'S_A': S_A,
        'S_B': S_B,
        'I_A': I_A,
        'I_B': I_B,
        'loading_A': loading_A,
        'loading_B': loading_B,
        'losses': total_losses,
        'efficiency': efficiency
    }


if __name__ == "__main__":
    results = calculate_transformer_load_distribution()
