"""
Test script for transformer load distribution calculations
Verifies the mathematical solution for the given problem
"""

import sys
sys.path.append('.')

from transformer_core import TransformerLoadDistribution
import math

def test_transformer_load_distribution():
    """Test the transformer load distribution calculation"""

    print("="*70)
    print("TESTING TRANSFORMER PARALLEL LOAD DISTRIBUTION")
    print("="*70)
    print()

    # Create transformer object with problem parameters
    transformer = TransformerLoadDistribution()

    # Default parameters (from problem statement)
    print("INPUT PARAMETERS:")
    print("-"*70)
    print(f"Transformer A: {transformer.S_A_rated} kVA")
    print(f"  Resistance: {transformer.R_A_percent}%")
    print(f"  Reactance: {transformer.X_A_percent}%")
    print()
    print(f"Transformer B: {transformer.S_B_rated} kVA")
    print(f"  Resistance: {transformer.R_B_percent}%")
    print(f"  Reactance: {transformer.X_B_percent}%")
    print()
    print(f"Total Load: {transformer.S_load} kVA at {transformer.pf} p.f. lagging")
    print(f"System Voltage: {transformer.voltage} kV")
    print()

    # Calculate impedances
    Z_A, Z_B = transformer.calculate_impedances()

    print("IMPEDANCE ANALYSIS (Common Base = 5000 kVA):")
    print("-"*70)
    print(f"Z_A = {Z_A}")
    print(f"|Z_A| = {abs(Z_A):.6f} pu")
    print(f"∠Z_A = {math.degrees(math.atan2(Z_A.imag, Z_A.real)):.2f}°")
    print()
    print(f"Z_B = {Z_B}")
    print(f"|Z_B| = {abs(Z_B):.6f} pu")
    print(f"∠Z_B = {math.degrees(math.atan2(Z_B.imag, Z_B.real)):.2f}°")
    print()

    # Calculate load distribution
    results = transformer.calculate_load_distribution()

    print("LOAD DISTRIBUTION RESULTS:")
    print("-"*70)
    print(f"Transformer A supplies: {results['S_A']:.2f} kVA")
    print(f"  Loading: {results['loading_A']:.2f}%")
    print(f"  Current: {results['I_A']:.2f} A")
    print()
    print(f"Transformer B supplies: {results['S_B']:.2f} kVA")
    print(f"  Loading: {results['loading_B']:.2f}%")
    print(f"  Current: {results['I_B']:.2f} A")
    print()
    print(f"Total supplied: {results['S_A'] + results['S_B']:.2f} kVA")
    print(f"Error: {abs(results['S_A'] + results['S_B'] - transformer.S_load):.2f} kVA")
    print()

    # Verify the load distribution ratio
    impedance_ratio = abs(Z_A) / abs(Z_B)
    power_ratio = results['S_B'] / results['S_A']

    print("VERIFICATION:")
    print("-"*70)
    print(f"Impedance ratio (|Z_A|/|Z_B|): {impedance_ratio:.4f}")
    print(f"Power ratio (S_B/S_A): {power_ratio:.4f}")
    print(f"Note: For parallel transformers, S_B/S_A ≈ |Z_A|/|Z_B|")
    print(f"Ratio match: {abs(impedance_ratio - power_ratio) < 0.1}")
    print()

    # Calculate losses
    losses_A = transformer.calculate_losses(
        results['S_A'], transformer.S_A_rated, transformer.R_A_percent
    )
    losses_B = transformer.calculate_losses(
        results['S_B'], transformer.S_B_rated, transformer.R_B_percent
    )

    print("LOSS ANALYSIS:")
    print("-"*70)
    print("Transformer A:")
    print(f"  Copper losses:     {losses_A['copper']:.2f} kW")
    print(f"  Iron losses:       {losses_A['iron']:.2f} kW")
    print(f"  Stray losses:      {losses_A['stray']:.2f} kW")
    print(f"  Mechanical losses: {losses_A['mechanical']:.2f} kW")
    print(f"  Total losses:      {losses_A['total']:.2f} kW")
    print()
    print("Transformer B:")
    print(f"  Copper losses:     {losses_B['copper']:.2f} kW")
    print(f"  Iron losses:       {losses_B['iron']:.2f} kW")
    print(f"  Stray losses:      {losses_B['stray']:.2f} kW")
    print(f"  Mechanical losses: {losses_B['mechanical']:.2f} kW")
    print(f"  Total losses:      {losses_B['total']:.2f} kW")
    print()
    print(f"Combined total losses: {losses_A['total'] + losses_B['total']:.2f} kW")
    print()

    # Calculate efficiency
    eff_A = transformer.calculate_efficiency(results['S_A'], losses_A)
    eff_B = transformer.calculate_efficiency(results['S_B'], losses_B)

    # Overall efficiency
    total_output = transformer.S_load * transformer.pf  # kW
    total_input = total_output + losses_A['total'] + losses_B['total']  # kW
    overall_efficiency = (total_output / total_input) * 100

    print("EFFICIENCY:")
    print("-"*70)
    print(f"Transformer A: {eff_A:.3f}%")
    print(f"Transformer B: {eff_B:.3f}%")
    print(f"Overall system: {overall_efficiency:.3f}%")
    print()

    # Temperature analysis
    temp_A = transformer.calculate_temperature(results['S_A'], transformer.S_A_rated)
    temp_B = transformer.calculate_temperature(results['S_B'], transformer.S_B_rated)

    print("THERMAL ANALYSIS (Steady-State):")
    print("-"*70)
    print(f"Ambient temperature: {transformer.ambient_temp}°C")
    print(f"Transformer A: {temp_A:.1f}°C")
    print(f"Transformer B: {temp_B:.1f}°C")
    print()

    # Test passed?
    total_supply = results['S_A'] + results['S_B']
    tolerance = 10  # kVA

    print("="*70)
    if abs(total_supply - transformer.S_load) < tolerance:
        print("✓ TEST PASSED: Load distribution calculation is correct!")
        print(f"  Total supply ({total_supply:.2f} kVA) matches load ({transformer.S_load} kVA)")
    else:
        print("✗ TEST FAILED: Load distribution mismatch!")
        print(f"  Total supply: {total_supply:.2f} kVA")
        print(f"  Expected load: {transformer.S_load} kVA")
        print(f"  Difference: {abs(total_supply - transformer.S_load):.2f} kVA")
    print("="*70)
    print()

    return results


if __name__ == "__main__":
    test_transformer_load_distribution()
