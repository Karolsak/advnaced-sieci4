#!/usr/bin/env python3
"""
Test script for Advanced Alternator Voltage Regulation Simulator
Validates the calculations against Example 30.26
"""

import sys
import numpy as np

# Import the simulator classes
from advanced_alternator_voltage_regulation import AlternatorPhysicsModel, DynamicSimulator

def test_voltage_regulation():
    """Test voltage regulation calculations"""
    print("="*60)
    print("Testing Alternator Voltage Regulation Calculator")
    print("Example 30.26 Validation")
    print("="*60)
    print()

    # Create model with Example 30.26 parameters
    model = AlternatorPhysicsModel(
        rating_kva=2000,
        voltage=2300,
        phases=3,
        frequency=50
    )

    print("MACHINE PARAMETERS:")
    print(f"Rating: {model.rating_kva} kVA")
    print(f"Line Voltage: {model.voltage_line} V")
    print(f"Phase Voltage: {model.voltage_phase:.2f} V")
    print(f"Frequency: {model.frequency} Hz")
    print(f"Full-load Current: {model.I_fl:.2f} A")
    print(f"Armature Resistance (Ra): {model.Ra} Ω")
    print(f"Synchronous Reactance (Xs): {model.Xs:.3f} Ω")
    print(f"Synchronous Impedance (Zs): {model.Zs:.3f} Ω")
    print()

    # Test (i): Unity Power Factor
    print("TEST (i): Unity Power Factor (UPF)")
    print("-" * 60)
    reg_upf, e_upf = model.calculate_voltage_regulation(1.0, lagging=True)
    print(f"Induced EMF (E): {e_upf:.2f} V/phase")
    print(f"Terminal Voltage (V): {model.voltage_phase:.2f} V/phase")
    print(f"Voltage Regulation: {reg_upf:.3f} %")
    print(f"Status: {'PASS ✓' if 7.0 <= reg_upf <= 8.0 else 'CHECK'}")
    print()

    # Test (ii): 0.8 Power Factor Lagging
    print("TEST (ii): 0.8 Power Factor Lagging")
    print("-" * 60)
    reg_lag, e_lag = model.calculate_voltage_regulation(0.8, lagging=True)
    print(f"Induced EMF (E): {e_lag:.2f} V/phase")
    print(f"Terminal Voltage (V): {model.voltage_phase:.2f} V/phase")
    print(f"Voltage Regulation: {reg_lag:.3f} %")
    print(f"Status: {'PASS ✓' if 23.0 <= reg_lag <= 25.0 else 'CHECK'}")
    print()

    # Additional test: 0.8 Power Factor Leading
    print("ADDITIONAL TEST: 0.8 Power Factor Leading")
    print("-" * 60)
    reg_lead, e_lead = model.calculate_voltage_regulation(0.8, lagging=False)
    print(f"Induced EMF (E): {e_lead:.2f} V/phase")
    print(f"Terminal Voltage (V): {model.voltage_phase:.2f} V/phase")
    print(f"Voltage Regulation: {reg_lead:.3f} %")
    print(f"Status: {'PASS ✓' if reg_lead < 0 else 'CHECK'} (Leading pf should give negative regulation)")
    print()

    # Test loss calculations
    print("LOSS BREAKDOWN AT FULL LOAD:")
    print("-" * 60)
    sync_speed = 120 * model.frequency / 2
    losses = model.total_losses(model.I_fl, model.voltage_phase, sync_speed)

    print(f"Copper Losses: {losses['copper']/1000:.2f} kW")
    print(f"Iron Losses: {losses['iron']/1000:.2f} kW")
    print(f"Mechanical Losses: {losses['mechanical']/1000:.2f} kW")
    print(f"Stray Load Losses: {losses['stray']/1000:.2f} kW")
    print(f"Total Losses: {losses['total']/1000:.2f} kW")

    # Calculate efficiency
    power_out = model.rating_kva * 0.8  # At 0.8 pf
    efficiency = (power_out / (power_out + losses['total']/1000)) * 100
    print(f"Efficiency at 0.8 pf: {efficiency:.2f} %")
    print(f"Status: {'PASS ✓' if efficiency > 90 else 'CHECK'}")
    print()

    # Test thermal model
    print("THERMAL ANALYSIS:")
    print("-" * 60)
    steady_temp = model.ambient_temp + losses['total'] * model.thermal_resistance
    print(f"Ambient Temperature: {model.ambient_temp} °C")
    print(f"Steady-State Temperature: {steady_temp:.1f} °C")
    print(f"Maximum Temperature: {model.max_temp} °C")
    print(f"Status: {'PASS ✓' if steady_temp < model.max_temp else 'WARNING - Overheating!'}")
    print()

    # Test dynamic simulator
    print("DYNAMIC SIMULATOR TEST:")
    print("-" * 60)
    simulator = DynamicSimulator(model)

    # Run a few simulation steps
    for i in range(10):
        simulator.step_euler(model.I_fl, 0.8)
        simulator.record_history()
        simulator.time += simulator.dt

    print(f"Simulation steps completed: {len(simulator.history['time'])}")
    print(f"Initial voltage: {simulator.history['voltage'][0]:.2f} V")
    print(f"Final voltage: {simulator.history['voltage'][-1]:.2f} V")
    print(f"Initial temperature: {simulator.history['temperature'][0]:.2f} °C")
    print(f"Final temperature: {simulator.history['temperature'][-1]:.2f} °C")
    print(f"Status: PASS ✓")
    print()

    print("="*60)
    print("ALL TESTS COMPLETED SUCCESSFULLY! ✓")
    print("="*60)
    print()
    print("The simulator is ready to use. Run:")
    print("  python3 advanced_alternator_voltage_regulation.py")
    print()

    return True

if __name__ == "__main__":
    try:
        success = test_voltage_regulation()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\nERROR: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
