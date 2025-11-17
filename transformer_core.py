"""
Core calculation module for transformer load distribution
(No GUI dependencies - can be imported anywhere)
"""

import numpy as np
import math
from scipy.integrate import solve_ivp


class TransformerLoadDistribution:
    """Core calculation engine for transformer load distribution"""

    def __init__(self):
        # Transformer A parameters
        self.S_A_rated = 2000  # kVA
        self.R_A_percent = 2.0  # %
        self.X_A_percent = 8.0  # %

        # Transformer B parameters
        self.S_B_rated = 4000  # kVA
        self.R_B_percent = 1.6  # %
        self.X_B_percent = 3.0  # %

        # Load parameters
        self.S_load = 5000  # kVA
        self.pf = 0.8  # power factor
        self.voltage = 11.0  # kV (assumed)

        # Physical parameters
        self.ambient_temp = 25  # °C
        self.cooling_coefficient = 0.05
        self.thermal_time_constant = 1800  # seconds

        # Results storage
        self.S_A = 0
        self.S_B = 0
        self.I_A = 0
        self.I_B = 0
        self.temp_A = self.ambient_temp
        self.temp_B = self.ambient_temp

    def calculate_impedances(self):
        """Calculate transformer impedances on common base"""
        # Common base = total load
        base_S = self.S_load

        # Convert to common base
        Z_A = complex(self.R_A_percent/100, self.X_A_percent/100) * (base_S / self.S_A_rated)
        Z_B = complex(self.R_B_percent/100, self.X_B_percent/100) * (base_S / self.S_B_rated)

        return Z_A, Z_B

    def calculate_load_distribution(self):
        """Calculate load distribution between parallel transformers"""
        Z_A, Z_B = self.calculate_impedances()

        # Calculate magnitudes
        Z_A_mag = abs(Z_A)
        Z_B_mag = abs(Z_B)

        # Load distribution (inversely proportional to impedance)
        total_admittance = 1/Z_A + 1/Z_B
        Y_A = 1/Z_A
        Y_B = 1/Z_B

        # Load current (total)
        I_load = self.S_load * 1000 / (math.sqrt(3) * self.voltage * 1000)  # Amps

        # Current distribution (admittance ratio)
        I_A_mag = abs(Y_A / total_admittance) * I_load
        I_B_mag = abs(Y_B / total_admittance) * I_load

        # Power distribution
        self.S_A = math.sqrt(3) * self.voltage * 1000 * I_A_mag / 1000  # kVA
        self.S_B = math.sqrt(3) * self.voltage * 1000 * I_B_mag / 1000  # kVA

        self.I_A = I_A_mag
        self.I_B = I_B_mag

        # Calculate loading percentages
        loading_A = (self.S_A / self.S_A_rated) * 100
        loading_B = (self.S_B / self.S_B_rated) * 100

        return {
            'S_A': self.S_A,
            'S_B': self.S_B,
            'I_A': self.I_A,
            'I_B': self.I_B,
            'loading_A': loading_A,
            'loading_B': loading_B,
            'Z_A': Z_A,
            'Z_B': Z_B
        }

    def calculate_losses(self, S_actual, S_rated, R_percent):
        """Calculate transformer losses"""
        # Copper losses (proportional to square of loading)
        loading_ratio = S_actual / S_rated
        copper_loss = (loading_ratio ** 2) * (R_percent / 100) * S_rated

        # Iron losses (approximately constant)
        iron_loss = 0.002 * S_rated  # Assumed 0.2% of rated power

        # Stray load losses
        stray_loss = 0.001 * S_rated * (loading_ratio ** 1.8)

        # Mechanical losses (friction, cooling)
        mechanical_loss = 0.001 * S_rated

        total_loss = copper_loss + iron_loss + stray_loss + mechanical_loss

        return {
            'copper': copper_loss,
            'iron': iron_loss,
            'stray': stray_loss,
            'mechanical': mechanical_loss,
            'total': total_loss
        }

    def calculate_temperature(self, S_actual, S_rated, time_step=1.0):
        """Calculate transformer temperature using thermal model"""
        loading_ratio = S_actual / S_rated

        # Simplified steady-state temperature
        delta_T_rated = 65  # °C temperature rise at rated load

        return delta_T_rated * (loading_ratio ** 2) + self.ambient_temp

    def calculate_efficiency(self, S_actual, losses):
        """Calculate transformer efficiency"""
        P_out = S_actual * self.pf  # kW
        P_in = P_out + losses['total']

        if P_in > 0:
            efficiency = (P_out / P_in) * 100
        else:
            efficiency = 0

        return efficiency
