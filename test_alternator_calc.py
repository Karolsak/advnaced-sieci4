#!/usr/bin/env python3
"""
Standalone test for alternator EMF calculations
Does not require GUI dependencies
"""

def test_alternator_calculations():
    """Test the alternator problem calculations"""
    print("="*70)
    print("ALTERNATOR EMF CALCULATION - STANDALONE TEST")
    print("="*70)
    print()

    # Given parameters
    print("GIVEN:")
    print("-" * 70)
    poles = 4
    frequency = 50.0  # Hz
    slots_per_pole = 15
    conductors_per_slot = 10
    winding_factor = 0.95
    terminal_voltage_star = 1825.0  # V (line voltage)
    speed_rpm = 1500.0  # RPM (calculated from 120*f/P)

    print(f"• Poles (P):                  {poles}")
    print(f"• Frequency (f):              {frequency} Hz")
    print(f"• Slots per pole:             {slots_per_pole}")
    print(f"• Conductors per slot:        {conductors_per_slot}")
    print(f"• Winding factor (Kw):        {winding_factor}")
    print(f"• Terminal voltage (star):    {terminal_voltage_star} V")
    print()

    # Calculate machine configuration
    total_slots = poles * slots_per_pole
    total_conductors = total_slots * conductors_per_slot
    conductors_per_phase = total_conductors // 3
    turns_per_phase = conductors_per_phase // 2

    print("MACHINE CONFIGURATION:")
    print("-" * 70)
    print(f"Total slots:                  {total_slots}")
    print(f"Total conductors (Z):         {total_conductors}")
    print(f"Conductors per phase:         {conductors_per_phase}")
    print(f"Turns per phase (Tph):        {turns_per_phase}")
    print()

    # Step 1: Calculate flux from star-connected terminal voltage
    print("STEP 1: Determine flux per pole from AC alternator")
    print("-" * 70)

    # For star connection: VL = √3 × Eph
    import math
    eph_star = terminal_voltage_star / math.sqrt(3)
    print(f"Line voltage (VL):            {terminal_voltage_star} V")
    print(f"Phase voltage (Eph):          {eph_star:.4f} V")
    print()

    # EMF equation for AC alternator: Eph = 4.44 × f × φ × Tph × Kw
    flux_per_pole = eph_star / (4.44 * frequency * turns_per_phase * winding_factor)
    print(f"Using: Eph = 4.44 × f × φ × Tph × Kw")
    print(f"Solving for φ:")
    print(f"φ = Eph / (4.44 × f × Tph × Kw)")
    print(f"φ = {eph_star:.4f} / (4.44 × {frequency} × {turns_per_phase} × {winding_factor})")
    print(f"φ = {flux_per_pole:.6f} Wb")
    print()

    # Verify the calculation
    eph_verify = 4.44 * frequency * flux_per_pole * turns_per_phase * winding_factor
    vl_verify = math.sqrt(3) * eph_verify
    print(f"Verification:")
    print(f"Calculated VL = √3 × Eph = {vl_verify:.2f} V")
    print(f"Matches given VL: {'✓ YES' if abs(vl_verify - terminal_voltage_star) < 1 else '✗ NO'}")
    print()

    # Step 2: Calculate synchronous speed
    print("STEP 2: Calculate synchronous speed")
    print("-" * 70)
    speed_rpm = (120 * frequency) / poles
    print(f"n = 120 × f / P")
    print(f"n = 120 × {frequency} / {poles}")
    print(f"n = {speed_rpm} RPM")
    print()

    # Step 3: Calculate EMF for lap-connected DC machine
    print("STEP 3: Calculate EMF for lap-connected winding (DC machine)")
    print("-" * 70)
    print()
    print("For DC machine with lap winding:")
    print(f"• Number of parallel paths (A) = P = {poles}")
    print(f"• Total conductors (Z) = {total_conductors}")
    print(f"• Speed (n) = {speed_rpm} RPM (same as AC)")
    print(f"• Flux per pole (φ) = {flux_per_pole:.6f} Wb (same as AC)")
    print()

    # EMF for DC machine: E = (φ × Z × n × P) / (60 × A)
    A_lap = poles  # For lap winding
    emf_lap = (flux_per_pole * total_conductors * speed_rpm * poles) / (60 * A_lap)

    print(f"EMF formula: E = (φ × Z × n × P) / (60 × A)")
    print(f"E = ({flux_per_pole:.6f} × {total_conductors} × {speed_rpm} × {poles}) / (60 × {A_lap})")
    print(f"E = {emf_lap:.2f} V")
    print()

    # Step 4: Wave winding comparison
    print("STEP 4: Wave winding comparison")
    print("-" * 70)
    A_wave = 2  # For wave winding, always 2
    emf_wave = (flux_per_pole * total_conductors * speed_rpm * poles) / (60 * A_wave)
    print(f"For wave winding (A = 2):")
    print(f"E = ({flux_per_pole:.6f} × {total_conductors} × {speed_rpm} × {poles}) / (60 × {A_wave})")
    print(f"E = {emf_wave:.2f} V")
    print()

    # Final answer
    print("="*70)
    print("FINAL ANSWER:")
    print("="*70)
    print()
    print(f"Question: If the windings are lap-connected as in a D.C. machine,")
    print(f"          what would be the e.m.f. between the brushes for the")
    print(f"          same speed and the same flux/pole?")
    print()
    print(f"Answer:   EMF between brushes = {emf_lap:.2f} V")
    print()
    print("="*70)
    print()

    # Summary
    print("SUMMARY:")
    print("-" * 70)
    print(f"AC Alternator (Star):         {terminal_voltage_star} V (line)")
    print(f"AC Alternator (Phase):        {eph_star:.2f} V")
    print(f"DC Machine (Lap):             {emf_lap:.2f} V")
    print(f"DC Machine (Wave):            {emf_wave:.2f} V")
    print()
    print(f"Flux per pole:                {flux_per_pole:.6f} Wb")
    print(f"Speed:                        {speed_rpm} RPM")
    print(f"Total conductors:             {total_conductors}")
    print(f"Turns per phase:              {turns_per_phase}")
    print()

    # Key insights
    print("KEY INSIGHTS:")
    print("-" * 70)
    print(f"• For the same flux and speed, DC machine produces different EMF")
    print(f"• Lap winding (A=P={poles}): {emf_lap:.2f} V")
    print(f"• Wave winding (A=2): {emf_wave:.2f} V")
    print(f"• Wave winding produces {emf_wave/emf_lap:.2f}× higher EMF than lap")
    print(f"• The key difference is the number of parallel paths (A)")
    print()

    return {
        'emf_lap': emf_lap,
        'emf_wave': emf_wave,
        'flux_per_pole': flux_per_pole,
        'speed_rpm': speed_rpm
    }


if __name__ == "__main__":
    results = test_alternator_calculations()
    print("✓ Calculation test completed successfully!")
    print(f"✓ Lap winding EMF: {results['emf_lap']:.2f} V")
    print(f"✓ Wave winding EMF: {results['emf_wave']:.2f} V")
