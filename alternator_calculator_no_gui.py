#!/usr/bin/env python3
"""
Alternator Voltage Regulation Calculator - No GUI Version
For testing and command-line use
"""

import numpy as np
from scipy.integrate import solve_ivp
import math


class AlternatorPhysicsModel:
    """Multi-physics model for alternator behavior"""

    def __init__(self, rating_kva, voltage, phases=3, frequency=50):
        self.rating_kva = rating_kva
        self.voltage_line = voltage
        self.voltage_phase = voltage / np.sqrt(3)
        self.phases = phases
        self.frequency = frequency
        self.omega = 2 * np.pi * frequency

        # Machine parameters
        self.Ra = 0.06  # Armature resistance per phase (Ω)
        self.Xs = 0.864  # Synchronous reactance per phase (Ω)
        self.Zs = np.sqrt(self.Ra**2 + self.Xs**2)

        # Full load current
        self.I_fl = (rating_kva * 1000) / (np.sqrt(3) * voltage)

        # Thermal parameters
        self.thermal_resistance = 0.05  # K/W
        self.thermal_capacitance = 5000  # J/K
        self.ambient_temp = 25  # °C
        self.max_temp = 155  # °C (Class F insulation)
        self.current_temp = self.ambient_temp

        # Mechanical parameters
        self.inertia = 50  # kg·m²
        self.friction_coeff = 0.1  # N·m·s
        self.rated_torque = (rating_kva * 1000) / (2 * np.pi * frequency / (self.phases/2))

        # Efficiency and losses
        self.iron_loss_coeff = 0.015
        self.friction_loss = 2000  # W
        self.windage_loss = 1500  # W
        self.stray_load_loss_factor = 0.01

    def calculate_voltage_regulation(self, power_factor, lagging=True):
        """Calculate voltage regulation at given power factor"""
        phi = np.arccos(power_factor)
        if not lagging:
            phi = -phi

        # Current at full load
        I = self.I_fl
        V = self.voltage_phase

        # Induced EMF per phase
        E = np.sqrt((V * np.cos(phi) + I * self.Ra)**2 +
                    (V * np.sin(phi) + I * self.Xs)**2)

        # Voltage regulation
        regulation = ((E - V) / V) * 100

        return regulation, E

    def copper_losses(self, current):
        """Calculate copper losses (I²R losses)"""
        return 3 * current**2 * self.Ra

    def iron_losses(self, voltage):
        """Calculate iron losses (core losses)"""
        return self.iron_loss_coeff * voltage**2

    def mechanical_losses(self, speed_rpm):
        """Calculate mechanical losses (friction + windage)"""
        speed_factor = (speed_rpm / (120 * self.frequency / 2))**2
        return self.friction_loss + self.windage_loss * speed_factor

    def stray_load_losses(self, current):
        """Calculate stray load losses"""
        return self.stray_load_loss_factor * (current / self.I_fl)**2 * self.rating_kva * 1000

    def total_losses(self, current, voltage, speed_rpm):
        """Calculate total losses"""
        P_cu = self.copper_losses(current)
        P_iron = self.iron_losses(voltage)
        P_mech = self.mechanical_losses(speed_rpm)
        P_stray = self.stray_load_losses(current)

        return {
            'copper': P_cu,
            'iron': P_iron,
            'mechanical': P_mech,
            'stray': P_stray,
            'total': P_cu + P_iron + P_mech + P_stray
        }

    def derating_factor(self, temperature):
        """Calculate derating factor based on temperature"""
        if temperature <= 100:
            return 1.0
        elif temperature <= self.max_temp:
            return 1.0 - 0.01 * (temperature - 100) / (self.max_temp - 100)
        else:
            return 0.5


def print_results():
    """Print detailed calculation results"""
    print("="*70)
    print("3-PHASE ALTERNATOR VOLTAGE REGULATION CALCULATOR")
    print("Example 30.26 - Detailed Solution")
    print("="*70)
    print()

    # Create model
    model = AlternatorPhysicsModel(
        rating_kva=2000,
        voltage=2300,
        phases=3,
        frequency=50
    )

    # Print given data
    print("GIVEN DATA:")
    print("-" * 70)
    print(f"Rating: {model.rating_kva} kVA, 3-phase, star-connected")
    print(f"Line Voltage: {model.voltage_line} V")
    print(f"Frequency: {model.frequency} Hz")
    print(f"Short-circuit current (Isc): 600 A")
    print(f"Open-circuit voltage (Voc): 900 V (line)")
    print(f"Resistance between terminals: 0.12 Ω")
    print()

    # Print calculated parameters
    print("CALCULATED MACHINE PARAMETERS:")
    print("-" * 70)
    print(f"Phase Voltage (V_ph): {model.voltage_phase:.2f} V")
    print(f"Full-load Current (I_fl): {model.I_fl:.2f} A")
    print(f"Armature Resistance per phase (Ra): {model.Ra} Ω")
    print(f"Open-circuit voltage per phase: {900/np.sqrt(3):.2f} V")
    print(f"Synchronous Impedance (Zs): {model.Zs:.3f} Ω")
    print(f"Synchronous Reactance (Xs): {model.Xs:.3f} Ω")
    print()

    # Calculate regulation at different power factors
    print("VOLTAGE REGULATION CALCULATIONS:")
    print("="*70)
    print()

    # (i) Unity Power Factor
    print("(i) AT UNITY POWER FACTOR (UPF):")
    print("-" * 70)
    reg_upf, e_upf = model.calculate_voltage_regulation(1.0, lagging=True)

    phi = 0
    V = model.voltage_phase
    I = model.I_fl
    Ra = model.Ra
    Xs = model.Xs

    print(f"Power Factor (cos φ): 1.0")
    print(f"Phase angle (φ): {phi}°")
    print()
    print("Induced EMF calculation:")
    print(f"E = √[(V×cos(φ) + I×Ra)² + (V×sin(φ) + I×Xs)²]")
    print(f"E = √[({V:.2f}×1.0 + {I:.2f}×{Ra})² + ({V:.2f}×0 + {I:.2f}×{Xs:.3f})²]")
    print(f"E = √[({V:.2f} + {I*Ra:.2f})² + ({I*Xs:.2f})²]")
    print(f"E = √[({V + I*Ra:.2f})² + ({I*Xs:.2f})²]")
    print(f"E = √[{(V + I*Ra)**2:.2f} + {(I*Xs)**2:.2f}]")
    print(f"E = √{(V + I*Ra)**2 + (I*Xs)**2:.2f}")
    print(f"E = {e_upf:.2f} V/phase")
    print()
    print(f"Voltage Regulation = [(E - V)/V] × 100")
    print(f"Voltage Regulation = [({e_upf:.2f} - {V:.2f})/{V:.2f}] × 100")
    print(f"Voltage Regulation = {reg_upf:.3f} %")
    print()

    # (ii) 0.8 Power Factor Lagging
    print("(ii) AT 0.8 POWER FACTOR LAGGING:")
    print("-" * 70)
    reg_lag, e_lag = model.calculate_voltage_regulation(0.8, lagging=True)

    phi = np.arccos(0.8) * 180 / np.pi
    cos_phi = 0.8
    sin_phi = 0.6

    print(f"Power Factor (cos φ): 0.8 lagging")
    print(f"Phase angle (φ): {phi:.2f}°")
    print(f"sin(φ): {sin_phi}")
    print()
    print("Induced EMF calculation:")
    print(f"E = √[(V×cos(φ) + I×Ra)² + (V×sin(φ) + I×Xs)²]")
    print(f"E = √[({V:.2f}×{cos_phi} + {I:.2f}×{Ra})² + ({V:.2f}×{sin_phi} + {I:.2f}×{Xs:.3f})²]")
    print(f"E = √[({V*cos_phi:.2f} + {I*Ra:.2f})² + ({V*sin_phi:.2f} + {I*Xs:.2f})²]")
    print(f"E = √[({V*cos_phi + I*Ra:.2f})² + ({V*sin_phi + I*Xs:.2f})²]")
    print(f"E = √[{(V*cos_phi + I*Ra)**2:.2f} + {(V*sin_phi + I*Xs)**2:.2f}]")
    print(f"E = √{(V*cos_phi + I*Ra)**2 + (V*sin_phi + I*Xs)**2:.2f}")
    print(f"E = {e_lag:.2f} V/phase")
    print()
    print(f"Voltage Regulation = [(E - V)/V] × 100")
    print(f"Voltage Regulation = [({e_lag:.2f} - {V:.2f})/{V:.2f}] × 100")
    print(f"Voltage Regulation = {reg_lag:.3f} %")
    print()

    # Additional: 0.8 Power Factor Leading
    print("ADDITIONAL: AT 0.8 POWER FACTOR LEADING:")
    print("-" * 70)
    reg_lead, e_lead = model.calculate_voltage_regulation(0.8, lagging=False)
    print(f"Induced EMF (E): {e_lead:.2f} V/phase")
    print(f"Voltage Regulation: {reg_lead:.3f} %")
    print(f"Note: Negative regulation indicates voltage rise (typical for leading pf)")
    print()

    # Loss analysis
    print("LOSS BREAKDOWN AT FULL LOAD (0.8 pf):")
    print("="*70)
    sync_speed = 120 * model.frequency / 2
    losses = model.total_losses(model.I_fl, model.voltage_phase, sync_speed)

    print(f"Synchronous Speed: {sync_speed:.0f} RPM")
    print()
    print(f"Copper Losses (3×I²×Ra): {losses['copper']/1000:.3f} kW")
    print(f"  = 3 × {model.I_fl:.2f}² × {model.Ra} = {losses['copper']:.2f} W")
    print()
    print(f"Iron Losses (Core losses): {losses['iron']/1000:.3f} kW")
    print(f"Mechanical Losses (Friction + Windage): {losses['mechanical']/1000:.3f} kW")
    print(f"Stray Load Losses: {losses['stray']/1000:.3f} kW")
    print()
    print(f"TOTAL LOSSES: {losses['total']/1000:.3f} kW")
    print()

    # Efficiency
    power_out = model.rating_kva * 0.8  # kW at 0.8 pf
    power_in = power_out + losses['total']/1000
    efficiency = (power_out / power_in) * 100

    print(f"EFFICIENCY AT FULL LOAD (0.8 pf):")
    print(f"Output Power: {power_out:.2f} kW")
    print(f"Input Power: {power_in:.2f} kW")
    print(f"Efficiency: {efficiency:.3f} %")
    print()

    # Thermal analysis
    print("THERMAL ANALYSIS:")
    print("="*70)
    steady_temp = model.ambient_temp + losses['total'] * model.thermal_resistance
    thermal_time_constant = model.thermal_capacitance * model.thermal_resistance

    print(f"Ambient Temperature: {model.ambient_temp}°C")
    print(f"Thermal Resistance: {model.thermal_resistance} K/W")
    print(f"Thermal Capacitance: {model.thermal_capacitance} J/K")
    print(f"Thermal Time Constant: {thermal_time_constant:.1f} seconds")
    print()
    print(f"Steady-State Temperature Rise: {losses['total'] * model.thermal_resistance:.1f}°C")
    print(f"Steady-State Temperature: {steady_temp:.1f}°C")
    print(f"Maximum Rated Temperature: {model.max_temp}°C (Class F insulation)")
    print()

    if steady_temp < model.max_temp:
        margin = model.max_temp - steady_temp
        print(f"✓ SAFE OPERATION - Temperature margin: {margin:.1f}°C")
    else:
        print(f"✗ WARNING - Exceeds maximum temperature!")

    print()
    print("="*70)
    print("SUMMARY OF RESULTS:")
    print("="*70)
    print(f"(i)  Voltage Regulation at UPF:           {reg_upf:.3f} %")
    print(f"(ii) Voltage Regulation at 0.8 pf lag:    {reg_lag:.3f} %")
    print(f"     Voltage Regulation at 0.8 pf lead:   {reg_lead:.3f} %")
    print(f"     Full-load Efficiency (0.8 pf):       {efficiency:.3f} %")
    print(f"     Total Losses at Full Load:           {losses['total']/1000:.3f} kW")
    print("="*70)


if __name__ == "__main__":
    print_results()
