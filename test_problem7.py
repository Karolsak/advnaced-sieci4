"""
Test script for Problem 7 solution
"""
import math

def problem_7():
    """
    Problem 7: DC shunt motor field resistance change for speed control
    """
    # Given data
    V = 250  # Supply voltage (V)
    Ra = 0.5  # Armature resistance (Ω)
    Rf1 = 250  # Initial field resistance (Ω)
    IL1 = 21  # Initial line current (A)
    N1 = 600  # Initial speed (rpm)
    N2 = 800  # Target speed (rpm)

    # Initial conditions
    If1 = V / Rf1  # Field current
    Ia1 = IL1 - If1  # Armature current
    Eb1 = V - Ia1 * Ra  # Back EMF

    print("=" * 80)
    print("PROBLEM 7: DC Shunt Motor Field Resistance Change for Speed Control")
    print("=" * 80)
    print()
    print("Given Data:")
    print(f"  Supply Voltage (V) = {V} V")
    print(f"  Armature Resistance (Ra) = {Ra} Ω")
    print(f"  Initial Field Resistance (Rf₁) = {Rf1} Ω")
    print(f"  Initial Line Current (IL₁) = {IL1} A")
    print(f"  Initial Speed (N₁) = {N1} rpm")
    print(f"  Target Speed (N₂) = {N2} rpm")
    print()

    print("Initial Operating Point:")
    print(f"  Field Current (If₁) = {If1:.3f} A")
    print(f"  Armature Current (Ia₁) = {Ia1:.3f} A")
    print(f"  Back EMF (Eb₁) = {Eb1:.2f} V")
    print()

    # Solve for If2 using quadratic equation
    a = Eb1 * (N2/N1) / If1
    b = -V
    c = If1 * Ia1 * Ra

    discriminant = b**2 - 4*a*c
    If2_solution1 = (-b + math.sqrt(discriminant)) / (2*a)
    If2_solution2 = (-b - math.sqrt(discriminant)) / (2*a)

    print(f"Quadratic coefficients: a={a:.4f}, b={b:.4f}, c={c:.4f}")
    print(f"Solution 1: If₂ = {If2_solution1:.4f} A")
    print(f"Solution 2: If₂ = {If2_solution2:.4f} A")
    print()

    # Check both solutions
    for i, If2 in enumerate([If2_solution1, If2_solution2], 1):
        if If2 > 0:
            Ia2 = (If1 * Ia1) / If2
            Eb2 = V - Ia2 * Ra
            IL2 = Ia2 + If2
            Rf2 = V / If2
            delta_Rf = Rf2 - Rf1

            print(f"Solution {i} Analysis:")
            print(f"  New Field Current (If₂) = {If2:.4f} A")
            print(f"  New Armature Current (Ia₂) = {Ia2:.2f} A")
            print(f"  New Line Current (IL₂) = {IL2:.2f} A")
            print(f"  New Back EMF (Eb₂) = {Eb2:.2f} V")
            print(f"  New Field Resistance (Rf₂) = {Rf2:.2f} Ω")
            print(f"  Change in Field Resistance (ΔRf) = {delta_Rf:.2f} Ω")

            # Verify constant torque
            torque_ratio = (If1*Ia1) / (If2*Ia2)
            print(f"  Torque ratio: {torque_ratio:.6f} (should be 1.0)")

            # Verify speed
            speed_ratio = (Eb2/Eb1) * (If1/If2)
            expected_ratio = N2/N1
            print(f"  Speed ratio from Eb: {speed_ratio:.6f}")
            print(f"  Expected ratio: {expected_ratio:.6f}")
            print(f"  Physical validity: {'✓ VALID' if Ia2 < 100 and Eb2 > 0 else '✗ INVALID'}")
            print()

    print("=" * 80)
    print("CONCLUSION:")
    print("=" * 80)

    # Choose the practical solution (Solution 1 is physically valid)
    If2 = If2_solution1  # Solution 1 gives reasonable currents
    Ia2 = (If1 * Ia1) / If2
    Rf2 = V / If2
    delta_Rf = Rf2 - Rf1

    print(f"The field resistance must be INCREASED by {delta_Rf:.2f} Ω")
    print(f"New field resistance: {Rf2:.2f} Ω")
    print(f"New field current: {If2:.4f} A")
    print(f"New armature current: {Ia2:.2f} A")
    print()

if __name__ == "__main__":
    problem_7()
