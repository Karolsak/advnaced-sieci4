"""
Core train braking energy calculations (no GUI dependencies)
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict


@dataclass
class TrainParameters:
    """Parameters for train braking simulation"""
    mass: float = 400000  # kg (400 tonnes)
    gradient: float = 1/70  # 1 in 70 gradient
    initial_speed: float = 22.22  # m/s (80 km/h)
    final_speed: float = 13.89  # m/s (50 km/h)
    time_duration: float = 120  # seconds
    tractive_resistance: float = 49  # N/tonne
    rotational_inertia_factor: float = 1.075  # 7.5% allowance
    motor_efficiency: float = 0.75  # 75%


class TrainBrakingCalculator:
    """Calculate energy returned during regenerative braking"""

    def __init__(self, params: TrainParameters):
        self.params = params

    def calculate_energy_returned(self) -> Dict[str, float]:
        """
        Calculate energy returned to the line during regenerative braking

        Returns:
            Dictionary containing all energy components and results
        """
        p = self.params

        # Convert speeds to m/s (already done in dataclass)
        v1 = p.initial_speed
        v2 = p.final_speed

        # Calculate mass including rotational inertia
        effective_mass = p.mass * p.rotational_inertia_factor

        # 1. Kinetic energy change (lost by train)
        ke_initial = 0.5 * effective_mass * v1**2
        ke_final = 0.5 * effective_mass * v2**2
        kinetic_energy_lost = ke_initial - ke_final

        # 2. Distance traveled during braking
        # Using average velocity method
        avg_velocity = (v1 + v2) / 2
        distance = avg_velocity * p.time_duration

        # 3. Potential energy change (train going downhill - energy added)
        # Height drop = distance * sin(angle)
        # For small angles: sin(angle) ≈ gradient
        height_drop = distance * p.gradient
        potential_energy_gained = p.mass * 9.81 * height_drop

        # 4. Energy lost to tractive resistance
        total_resistance_force = p.tractive_resistance * (p.mass / 1000)  # N/tonne * tonnes
        resistance_energy = total_resistance_force * distance

        # 5. Total energy available for regeneration (before motor efficiency)
        # Energy balance: KE_lost + PE_gained - Resistance = Energy to braking system
        energy_to_braking = kinetic_energy_lost + potential_energy_gained - resistance_energy

        # 6. Energy returned to line (after motor efficiency)
        energy_returned = energy_to_braking * p.motor_efficiency

        # Calculate braking power (average)
        avg_braking_power = energy_to_braking / p.time_duration
        avg_power_returned = energy_returned / p.time_duration

        return {
            'kinetic_energy_lost': kinetic_energy_lost / 1e6,  # MJ
            'potential_energy_gained': potential_energy_gained / 1e6,  # MJ
            'resistance_energy': resistance_energy / 1e6,  # MJ
            'energy_to_braking': energy_to_braking / 1e6,  # MJ
            'energy_returned': energy_returned / 1e6,  # MJ
            'avg_braking_power': avg_braking_power / 1e3,  # kW
            'avg_power_returned': avg_power_returned / 1e3,  # kW
            'distance': distance,  # m
            'height_drop': height_drop,  # m
            'efficiency': (energy_returned / energy_to_braking) * 100 if energy_to_braking > 0 else 0  # %
        }


def main():
    """Command-line interface for calculations"""
    print("=" * 80)
    print("TRAIN BRAKING ENERGY CALCULATOR")
    print("=" * 80)
    print()

    # Default problem
    params = TrainParameters()
    calc = TrainBrakingCalculator(params)
    results = calc.calculate_energy_returned()

    print("PROBLEM:")
    print("-" * 80)
    print(f"A {params.mass/1000:.0f}-tonne train travels down a gradient of 1 in {1/params.gradient:.0f}")
    print(f"for {params.time_duration:.0f} seconds during which period its speed is reduced")
    print(f"from {params.initial_speed*3.6:.0f} km/h to {params.final_speed*3.6:.0f} km/h by regenerative braking.")
    print(f"Tractive resistance is {params.tractive_resistance:.0f} N/t and allowance for")
    print(f"rotational inertia is {(params.rotational_inertia_factor-1)*100:.1f}%. Overall efficiency")
    print(f"of motors is {params.motor_efficiency*100:.0f}%.")
    print()

    print("SOLUTION:")
    print("-" * 80)
    print(f"Kinetic Energy Lost:           {results['kinetic_energy_lost']:.2f} MJ")
    print(f"Potential Energy Gained:       {results['potential_energy_gained']:.2f} MJ")
    print(f"Energy Lost to Resistance:     {results['resistance_energy']:.2f} MJ")
    print(f"Energy to Braking System:      {results['energy_to_braking']:.2f} MJ")
    print()
    print(f"ENERGY RETURNED TO LINE:       {results['energy_returned']:.2f} MJ ✓")
    print()
    print(f"Average Braking Power:         {results['avg_braking_power']:.2f} kW")
    print(f"Average Power Returned:        {results['avg_power_returned']:.2f} kW")
    print()
    print("=" * 80)


if __name__ == "__main__":
    main()
