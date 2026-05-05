import numpy as np
from thermodynamics import (
    CH4RealGasProps, MethaloxProperties,
    CEA_MethaloxCombustion, PropellantPhysics,
)


class Turbopump:
    """
    Modello dinamico turbopompa centrifuga con espansione termodinamica reale della turbina.
    Bilancio coppia: tau_turbina - tau_pompa = I * d(omega)/dt
    Potenza turbina: P = mdot * cp * T_in * eta_t * [1 - (P_out/P_in)^((gamma-1)/gamma)]
    """
    def __init__(self, inertia, pump_k1, pump_k2, fluid, npsh_coeff=0.7e-8):
        self.I             = inertia
        self.pump_k1       = pump_k1
        self.pump_k2       = pump_k2
        self.fluid         = fluid
        self.npsh_coeff    = npsh_coeff
        self.is_cavitating = False

    def get_derivative(self, rpm, p_out_pump_bar, p_in_turb_bar, p_out_turb_bar,
                       t_in_turb, mdot_turb, gamma, cp_gas,
                       starter_torque, p_inlet_pump_bar, t_prop):

        # Clamp RPM: il solver BDF prova valori negativi durante la stima Jacobiana.
        # Con rpm < 0, pump_k2*rpm*P è negativo → tau_p < 0 → dw_dt esplode.
        rpm = max(0.0, rpm)

        # ── Lato pompa ────────────────────────────────────────────────────────
        p_v        = PropellantPhysics.get_vapor_pressure_bar(t_prop, self.fluid)
        npsh_avail = p_inlet_pump_bar - p_v
        npsh_req   = self.npsh_coeff * rpm**2

        cav_factor = 1.0
        self.is_cavitating = False
        if npsh_avail < npsh_req:
            self.is_cavitating = True
            cav_factor = max(0.01, 1.0 - (npsh_req - npsh_avail) * 2.0)

        tau_p = (self.pump_k1 * rpm**2 + self.pump_k2 * rpm * p_out_pump_bar) * cav_factor

        # Freno idrodinamico oltre i limiti di progetto
        if rpm > 38000.0:
            tau_p += 100.0 * (rpm - 38000.0)**2

        # ── Lato turbina (termodinamica reale) ────────────────────────────────
        if p_in_turb_bar > p_out_turb_bar and mdot_turb > 0.01 and t_in_turb > 50.0:
            pr        = max(1e-6, p_out_turb_bar / p_in_turb_bar)
            eta_t     = 0.70
            exp       = (gamma - 1.0) / gamma
            power_W   = mdot_turb * cp_gas * t_in_turb * eta_t * (1.0 - pr**exp)
            omega     = max(rpm * np.pi / 30.0, 1.0)
            tau_t_thermo = power_W / omega
        else:
            tau_t_thermo = 0.0

        # Starter: soffiaggio idraulico prima dell'accensione
        tau_t = tau_t_thermo + starter_torque

        # Freno aerodinamico oltre il regime nominale
        tau_t *= max(0.0, 1.0 - (rpm / 38000.0))

        # Conversione rad/s² → RPM/s: d(RPM)/dt = (30/π) · (τ/I)
        # La variabile di stato è in RPM, quindi la derivata deve esserlo.
        dw_dt = (tau_t - tau_p) / self.I * (30.0 / np.pi)

        if rpm <= 0.1 and dw_dt < 0:
            return 0.0
        return dw_dt


class CoolingJacket:
    def __init__(self, mass, t_inlet, h_a_base, t_flame_hot,
                 h_cool_nom: float = 80000.0, mdot_nom: float = 160.0):
        self.mass        = mass           
        self.t_inlet     = t_inlet        
        self.h_a_base    = h_a_base       
        self.t_flame_hot = t_flame_hot    
        self.h_cool_nom  = h_cool_nom     
        self.mdot_nom    = mdot_nom       

    def get_derivative(self, t_coolant, p_mcc_pa, mdot_coolant, is_ignited, t_amb, k_amb,
                       coolant_p_bar: float = 200.0):
        cp_current = MethaloxProperties.get_ch4_cp(t_coolant, coolant_p_bar)

        if is_ignited:
            h_gas = self.h_a_base * (p_mcc_pa / 101325.0)**0.8
            mdot_safe = max(mdot_coolant, 0.0)
            h_cool = self.h_cool_nom * (mdot_safe / self.mdot_nom)**0.8

            if h_cool > 0.0:
                h_eff = (h_gas * h_cool) / (h_gas + h_cool)
            else:
                h_eff = 0.0

            q_combustion = h_eff * (self.t_flame_hot - t_coolant)
        else:
            q_combustion = 0.0

        q_ambient = k_amb * (t_amb - t_coolant)
        q_removed  = mdot_coolant * cp_current * (t_coolant - self.t_inlet)

        return (q_combustion + q_ambient - q_removed) / (self.mass * cp_current)


class CombustionChamber:
    def __init__(self, volume, a_throat, eps_nozzle=40.0, eta_cstar=0.97):
        self.volume    = volume      
        self.a_t       = a_throat    
        self.eps       = eps_nozzle  
        self.eta_cstar = eta_cstar   
        self.p_atm     = 101325.0    

    def _of_ratio(self, mdot_ox, mdot_f):
        return mdot_ox / max(0.01, mdot_f)

    def get_c_star_eff(self, mdot_ox, mdot_f, p_mcc_pa, t_fuel=None):
        of    = self._of_ratio(mdot_ox, mdot_f)
        p_bar = p_mcc_pa / 1e5
        return CEA_MethaloxCombustion.get_c_star(of, p_bar, t_fuel=t_fuel) * self.eta_cstar

    def get_exhaust_mass_flow(self, p_mcc_pa, is_ignited, c_star_eff,
                              p_back_pa=None):
        p_back = p_back_pa if p_back_pa is not None else self.p_atm
        if p_mcc_pa <= p_back:
            return 0.0
        cs = max(c_star_eff if is_ignited else 300.0, 1.0)
        dp = max(0.0, p_mcc_pa - p_back)
        smoothing = min(1.0, dp / 50000.0)
        return smoothing * (p_mcc_pa * self.a_t) / cs

    def get_thrust_kn(self, p_mcc_pa, is_ignited, mdot_ox, mdot_f):
        gauge_p = max(0.0, p_mcc_pa - self.p_atm)
        if not is_ignited:
            return 0.2 * gauge_p * self.a_t / 1000.0
        of    = self._of_ratio(mdot_ox, mdot_f)
        gamma = CEA_MethaloxCombustion.get_gamma(of)
        cf    = CEA_MethaloxCombustion.compute_cf(gamma, self.eps, p_mcc_pa, self.p_atm)
        return cf * gauge_p * self.a_t / 1000.0

    def get_derivative(self, p_mcc_pa, mdot_ox, mdot_f, is_ignited,
                       p_back_pa=None):
        p_back   = p_back_pa if p_back_pa is not None else self.p_atm
        cs_eff   = self.get_c_star_eff(mdot_ox, mdot_f, p_mcc_pa)
        mdot_in  = mdot_ox + mdot_f
        mdot_out = self.get_exhaust_mass_flow(p_mcc_pa, is_ignited, cs_eff,
                                              p_back_pa=p_back)

        if is_ignited and mdot_f > 0.01:
            of    = self._of_ratio(mdot_ox, mdot_f)
            p_bar = p_mcc_pa / 1e5
            t_fl  = CEA_MethaloxCombustion.get_t_ad(of, p_bar)
            mw    = CEA_MethaloxCombustion.get_mw(of)
            r_t   = (8314.0 / mw) * t_fl
        else:
            r_t = 1.0e5

        dP_dt = (r_t / self.volume) * (mdot_in - mdot_out)
        if p_mcc_pa <= p_back and dP_dt < 0:
            return 0.0
        return dP_dt
