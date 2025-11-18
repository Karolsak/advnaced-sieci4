#!/usr/bin/env python3
"""
Test script for alternator EMF calculations
Verifies the solution to the given problem
"""

import sys
sys.path.insert(0, '.')

from alternator_emf_simulator import AlternatorParameters, AlternatorCalculator

def test_alternator_problem():
    """Test the alternator problem from the assignment"""
    print("="*70)
    print("ALTERNATOR EMF CALCULATION TEST")
    print("="*70)
    print()

    # Problem parameters
    print("GIVEN:")
    print("-" * 70)
    print("• 4-pole, 50-Hz, star-connected alternator")
    print("• 15 slots per pole")
    print("• 10 conductors per slot")
    print("• Winding factor: 0.95")
    print("• Terminal EMF (star): 1825 V")
    print()

    # Create parameters
    params = AlternatorParameters(
        poles=4,
        frequency=50.0,
        slots_per_pole=15,
        conductors_per_slot=10,
        winding_factor=0.95,
        speed_rpm=1500.0
    )

    # Create calculator
    calc = AlternatorCalculator(params)

    # Display machine configuration
    print("MACHINE CONFIGURATION:")
    print("-" * 70)
    print(f"Total Slots:                  {calc.total_slots}")
    print(f"Total Conductors:             {calc.total_conductors}")
    print(f"Conductors per Phase:         {calc.conductors_per_phase}")
    print(f"Turns per Phase:              {calc.turns_per_phase}")
    print()

    # Step 1: Calculate flux from given terminal voltage
    print("STEP 1: Calculate flux per pole from given terminal voltage")
    print("-" * 70)

    # For star connection: Line voltage = √3 × Phase voltage
    vl_given = 1825.0
    eph_given = vl_given / (3**0.5)
    print(f"Given Line Voltage (VL):      {vl_given} V")
    print(f"Phase Voltage (Eph):          {eph_given:.2f} V")
    print()

    # Calculate flux from EMF equation: Eph = 4.44 × f × φ × Tph × Kw
    flux_calculated = eph_given / (4.44 * params.frequency * calc.turns_per_phase * params.winding_factor)
    print(f"Flux per pole (φ):            {flux_calculated:.6f} Wb")
    print()

    # Update flux in parameters
    params.flux_per_pole = flux_calculated
    calc = AlternatorCalculator(params)

    # Verify star connection EMF
    eph_star, vl_star = calc.calculate_star_emf()
    print("VERIFICATION - Star Connection:")
    print(f"Calculated Phase Voltage:     {eph_star:.2f} V")
    print(f"Calculated Line Voltage:      {vl_star:.2f} V")
    print(f"Match with given voltage:     {'✓ YES' if abs(vl_star - 1825) < 1 else '✗ NO'}")
    print()

    # Step 2: Calculate EMF for lap-connected DC machine
    print("STEP 2: Calculate EMF for lap-connected winding (DC machine)")
    print("-" * 70)
    print(f"Speed:                        {params.speed_rpm} RPM (same as AC)")
    print(f"Flux per pole:                {params.flux_per_pole:.6f} Wb (same as AC)")
    print(f"Number of parallel paths (A): {params.poles} (lap winding)")
    print()

    emf_lap = calc.calculate_lap_winding_emf()
    print(f"EMF between brushes (lap):    {emf_lap:.2f} V")
    print()

    # Step 3: Also show wave winding for comparison
    print("COMPARISON - Wave Winding:")
    print("-" * 70)
    emf_wave = calc.calculate_wave_winding_emf()
    print(f"Number of parallel paths (A): 2 (wave winding)")
    print(f"EMF between brushes (wave):   {emf_wave:.2f} V")
    print()

    # Final answer
    print("="*70)
    print("ANSWER:")
    print("="*70)
    print(f"For lap-connected winding at the same speed and flux:")
    print(f"EMF between brushes = {emf_lap:.2f} V")
    print("="*70)
    print()

    # Additional analysis
    print("ADDITIONAL ANALYSIS:")
    print("-" * 70)
    print(f"AC to DC EMF ratio (lap):     {emf_lap / vl_star:.4f}")
    print(f"AC to DC EMF ratio (wave):    {emf_wave / vl_star:.4f}")
    print()

    # Verify using formula
    print("FORMULA VERIFICATION:")
    print("-" * 70)
    print("DC Machine EMF: E = (φ × Z × n × P) / (60 × A)")
    Z = calc.total_conductors
    n = params.speed_rpm
    P = params.poles
    phi = params.flux_per_pole
    A_lap = params.poles

    emf_manual = (phi * Z * n * P) / (60 * A_lap)
    print(f"φ = {phi:.6f} Wb")
    print(f"Z = {Z} conductors")
    print(f"n = {n} RPM")
    print(f"P = {P} poles")
    print(f"A = {A_lap} (lap winding)")
    print(f"E = ({phi:.6f} × {Z} × {n} × {P}) / (60 × {A_lap})")
    print(f"E = {emf_manual:.2f} V")
    print(f"Match with calculator: {'✓ YES' if abs(emf_manual - emf_lap) < 0.01 else '✗ NO'}")
    print()

    return emf_lap


if __name__ == "__main__":
    result = test_alternator_problem()
    print(f"\n✓ Test completed successfully!")
    print(f"✓ Answer: {result:.2f} V")
