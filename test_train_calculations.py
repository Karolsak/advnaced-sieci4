"""
Test script to verify train braking energy calculations
"""

import sys
sys.path.append('/home/user/advnaced-sieci4')

from train_braking_energy_simulator import TrainParameters, TrainBrakingCalculator

def test_train_braking_calculation():
    """
    Test the train braking energy calculation with the given problem:
    - 400-tonne train
    - Gradient: 1 in 70 (downward)
    - Speed reduction: 80 km/h → 50 km/h over 120 seconds
    - Tractive resistance: 49 N/t
    - Rotational inertia: 7.5%
    - Motor efficiency: 75%
    """
    print("=" * 80)
    print("TRAIN BRAKING ENERGY CALCULATION TEST")
    print("=" * 80)
    print()

    # Create parameters
    params = TrainParameters(
        mass=400000,  # 400 tonnes = 400,000 kg
        gradient=1/70,  # 1 in 70 gradient
        initial_speed=80/3.6,  # 80 km/h = 22.22 m/s
        final_speed=50/3.6,  # 50 km/h = 13.89 m/s
        time_duration=120,  # seconds
        tractive_resistance=49,  # N/tonne
        rotational_inertia_factor=1.075,  # 7.5% allowance
        motor_efficiency=0.75  # 75%
    )

    # Create calculator
    calc = TrainBrakingCalculator(params)

    # Calculate results
    results = calc.calculate_energy_returned()

    # Display results
    print("INPUT PARAMETERS:")
    print("-" * 80)
    print(f"Mass:                          {params.mass/1000:.1f} tonnes")
    print(f"Initial Speed:                 {params.initial_speed*3.6:.1f} km/h ({params.initial_speed:.2f} m/s)")
    print(f"Final Speed:                   {params.final_speed*3.6:.1f} km/h ({params.final_speed:.2f} m/s)")
    print(f"Time Duration:                 {params.time_duration:.1f} seconds")
    print(f"Gradient:                      1 in {1/params.gradient:.1f}")
    print(f"Tractive Resistance:           {params.tractive_resistance:.1f} N/tonne")
    print(f"Rotational Inertia Factor:     {(params.rotational_inertia_factor-1)*100:.1f}%")
    print(f"Motor Efficiency:              {params.motor_efficiency*100:.1f}%")
    print()

    print("CALCULATION STEPS:")
    print("-" * 80)
    print()

    # Step-by-step calculation
    print("Step 1: Calculate Kinetic Energy Change")
    print(f"  Effective mass = {params.mass:.0f} kg × {params.rotational_inertia_factor:.3f}")
    print(f"                 = {params.mass * params.rotational_inertia_factor:.0f} kg")
    print(f"  KE_initial     = 0.5 × {params.mass * params.rotational_inertia_factor:.0f} × {params.initial_speed:.2f}²")
    print(f"                 = {0.5 * params.mass * params.rotational_inertia_factor * params.initial_speed**2 / 1e6:.2f} MJ")
    print(f"  KE_final       = 0.5 × {params.mass * params.rotational_inertia_factor:.0f} × {params.final_speed:.2f}²")
    print(f"                 = {0.5 * params.mass * params.rotational_inertia_factor * params.final_speed**2 / 1e6:.2f} MJ")
    print(f"  ΔKE (lost)     = {results['kinetic_energy_lost']:.2f} MJ")
    print()

    print("Step 2: Calculate Distance Traveled")
    avg_velocity = (params.initial_speed + params.final_speed) / 2
    print(f"  Average velocity = ({params.initial_speed:.2f} + {params.final_speed:.2f}) / 2")
    print(f"                   = {avg_velocity:.2f} m/s")
    print(f"  Distance         = {avg_velocity:.2f} × {params.time_duration:.1f}")
    print(f"                   = {results['distance']:.2f} m")
    print()

    print("Step 3: Calculate Potential Energy Change (descending)")
    print(f"  Height drop   = {results['distance']:.2f} × {params.gradient:.6f}")
    print(f"                = {results['height_drop']:.2f} m")
    print(f"  PE gained     = {params.mass:.0f} × 9.81 × {results['height_drop']:.2f}")
    print(f"                = {results['potential_energy_gained']:.2f} MJ")
    print()

    print("Step 4: Calculate Energy Lost to Resistance")
    total_resistance = params.tractive_resistance * (params.mass / 1000)
    print(f"  Total resistance force = {params.tractive_resistance:.1f} × {params.mass/1000:.1f}")
    print(f"                         = {total_resistance:.0f} N")
    print(f"  Energy to resistance   = {total_resistance:.0f} × {results['distance']:.2f}")
    print(f"                         = {results['resistance_energy']:.2f} MJ")
    print()

    print("Step 5: Calculate Energy to Braking System")
    print(f"  Energy to braking = KE_lost + PE_gained - Resistance")
    print(f"                    = {results['kinetic_energy_lost']:.2f} + {results['potential_energy_gained']:.2f} - {results['resistance_energy']:.2f}")
    print(f"                    = {results['energy_to_braking']:.2f} MJ")
    print()

    print("Step 6: Calculate Energy Returned to Line")
    print(f"  Energy returned = {results['energy_to_braking']:.2f} × {params.motor_efficiency:.2f}")
    print(f"                  = {results['energy_returned']:.2f} MJ")
    print()

    print("=" * 80)
    print(f"FINAL ANSWER: {results['energy_returned']:.2f} MJ returned to the line")
    print("=" * 80)
    print()

    print("ADDITIONAL INFORMATION:")
    print("-" * 80)
    print(f"Average Braking Power:         {results['avg_braking_power']:.2f} kW")
    print(f"Average Power Returned:        {results['avg_power_returned']:.2f} kW")
    print(f"Overall System Efficiency:     {results['efficiency']:.2f}%")
    print()

    # Verification
    print("VERIFICATION:")
    print("-" * 80)
    expected_range_min = 10.0  # MJ
    expected_range_max = 12.0  # MJ

    if expected_range_min <= results['energy_returned'] <= expected_range_max:
        print("✓ Result is within expected range")
        print(f"  Expected: {expected_range_min:.2f} - {expected_range_max:.2f} MJ")
        print(f"  Actual:   {results['energy_returned']:.2f} MJ")
        return True
    else:
        print("✗ Result is outside expected range")
        print(f"  Expected: {expected_range_min:.2f} - {expected_range_max:.2f} MJ")
        print(f"  Actual:   {results['energy_returned']:.2f} MJ")
        return False


if __name__ == "__main__":
    success = test_train_braking_calculation()
    sys.exit(0 if success else 1)
