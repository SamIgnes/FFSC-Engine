import numpy as np
from scipy.integrate import solve_ivp

from core.thermodynamics import CH4RealGasProps, CEA_MethaloxCombustion
from core.avionics import ProportionalValve
from core.subsystems import Turbopump, CoolingJacket, CombustionChamber
from config import (
    K_HEAD,
    RHO_LOX, RHO_LCH4_NOM,
    TANK_P_INIT_BAR, TANK_T_OX_INIT, TANK_T_F_INIT, TANK_H_OX_M, TANK_H_F_M,
    TP_OX_INERTIA, TP_OX_PUMP_K1, TP_OX_PUMP_K2, TP_OX_NPSH,
    TP_F_INERTIA,  TP_F_PUMP_K1,  TP_F_PUMP_K2,  TP_F_NPSH,
    VALVE_TAU_MOV, VALVE_TAU_MFV, VALVE_TAU_ORHC, VALVE_TAU_FRHC, VALVE_TAU_AUTO,
    R_VALVE_OX_K, R_VALVE_F_K, R_LINE_OX, R_LINE_F, R_JACKET_BASE,
    R_BLEED_OX_FRHC, R_BLEED_F_ORHC,
    CD_INJ_LIQUID, CD_INJ_GAS,
    A_INJ_OX_ORHC, A_INJ_F_FRHC, A_INJ_OX_FRHC, A_INJ_F_ORHC,
    A_INJ_GAS_OX, A_INJ_GAS_F,
    ORHC_VOLUME, ORHC_A_THROAT, ORHC_ETA_CSTAR,
    FRHC_VOLUME, FRHC_A_THROAT, FRHC_ETA_CSTAR,
    MCC_VOLUME, MCC_A_THROAT, MCC_EPS_NOZZLE, MCC_ETA_CSTAR,
    P_BACK_FACTOR,
    JACKET_MASS, JACKET_T_INLET, JACKET_H_A_BASE, JACKET_T_FLAME,
    JACKET_H_COOL, JACKET_MDOT_NOM,
    AUTOG_RATE, AUTOG_BLEED_OX, AUTOG_BLEED_F,
    STARTER_TORQUE, STARTER_TAU_BAR,
)


class FFSCC_Engine:
    """
    Vettore di stato (14 componenti):
        [0]  P_mcc     [Pa]
        [1]  w_ox      [RPM]
        [2]  w_f       [RPM]
        [3]  th_mov    [-]   (apertura valvola ossidante)
        [4]  th_mfv    [-]   (apertura valvola combustibile)
        [5]  v_orhc    [-]   (bypass pre-burner ox)
        [6]  v_frhc    [-]   (bypass pre-burner fuel)
        [7]  T_cool    [K]
        [8]  P_tank_ox [bar]
        [9]  P_tank_f  [bar]
        [10] v_auto_ox [-]
        [11] v_auto_f  [-]
        [12] P_orhc    [Pa]
        [13] P_frhc    [Pa]
    """

    def __init__(self):
        self.valve_mov     = ProportionalValve(tau=VALVE_TAU_MOV)
        self.valve_mfv     = ProportionalValve(tau=VALVE_TAU_MFV)
        self.valve_orhc    = ProportionalValve(tau=VALVE_TAU_ORHC)
        self.valve_frhc    = ProportionalValve(tau=VALVE_TAU_FRHC)
        self.valve_auto_ox = ProportionalValve(tau=VALVE_TAU_AUTO)
        self.valve_auto_f  = ProportionalValve(tau=VALVE_TAU_AUTO)

        self.R_jacket_base = R_JACKET_BASE

        self.orhc_chamber = CombustionChamber(volume=ORHC_VOLUME, a_throat=ORHC_A_THROAT, eta_cstar=ORHC_ETA_CSTAR)
        self.frhc_chamber = CombustionChamber(volume=FRHC_VOLUME, a_throat=FRHC_A_THROAT, eta_cstar=FRHC_ETA_CSTAR)

        self.tp_ox = Turbopump(
            inertia=TP_OX_INERTIA, pump_k1=TP_OX_PUMP_K1, pump_k2=TP_OX_PUMP_K2,
            fluid="LOX", npsh_coeff=TP_OX_NPSH
        )
        self.tp_fuel = Turbopump(
            inertia=TP_F_INERTIA, pump_k1=TP_F_PUMP_K1, pump_k2=TP_F_PUMP_K2,
            fluid="CH4", npsh_coeff=TP_F_NPSH
        )

        self.jacket = CoolingJacket(
            mass=JACKET_MASS, t_inlet=JACKET_T_INLET, h_a_base=JACKET_H_A_BASE,
            t_flame_hot=JACKET_T_FLAME, h_cool_nom=JACKET_H_COOL, mdot_nom=JACKET_MDOT_NOM,
        )
        self.mcc = CombustionChamber(volume=MCC_VOLUME, a_throat=MCC_A_THROAT, eps_nozzle=MCC_EPS_NOZZLE, eta_cstar=MCC_ETA_CSTAR)

        self.state = np.array([
            101325.0,        # P_mcc
            1.0,             # w_ox
            1.0,             # w_f
            0.0,             # th_mov
            0.0,             # th_mfv
            0.0,             # v_orhc
            0.0,             # v_frhc
            113.0,           # T_cool
            TANK_P_INIT_BAR, # P_tank_ox
            TANK_P_INIT_BAR, # P_tank_f
            0.0,             # v_auto_ox
            0.0,             # v_auto_f
            101325.0,        # P_orhc
            101325.0,        # P_frhc
        ])

        self.cmd_th_mov    = 0.0
        self.cmd_th_mfv    = 0.0
        self.cmd_v_orhc    = 0.0
        self.cmd_v_frhc    = 0.0
        self.cmd_v_auto_ox = 0.0
        self.cmd_v_auto_f  = 0.0
        
        self.is_ignited    = False
        self.current_time  = 0.0
        self.t_orhc_current  = 300.0
        self.t_frhc_current  = 300.0
        self.of_orhc_current = 0.0
        self.of_frhc_current = 0.0
        self.mdot_ox_last       = 0.0
        self.mdot_f_last        = 0.0
        self.mdot_out_orhc_last = 0.0
        self.mdot_out_frhc_last = 0.0
        self.dp_inj_ox_bar      = 0.0   # iniettori MCC lato ox
        self.dp_inj_f_bar       = 0.0   # iniettori MCC lato fuel
        # Iniettori precombustori (ΔP tra manifold pompa e camera preburner)
        self.dp_inj_ox_orhc_bar = 0.0   # LOX → ORHC
        self.dp_inj_f_frhc_bar  = 0.0   # CH4 → FRHC
        self.dp_inj_ox_frhc_bar = 0.0   # LOX bleed → FRHC
        self.dp_inj_f_orhc_bar  = 0.0   # CH4 bleed → ORHC
        self.p_man_ox_bar       = 0.0   # pressione manifold LOX (lato liquido, prima iniettori ORHC)
        self.p_man_f_bar        = 0.0   # pressione manifold CH4 (lato liquido, prima iniettori FRHC)
        self.t_tank_f  = TANK_T_F_INIT
        self.t_tank_ox = TANK_T_OX_INIT
        self.h_tank_ox_m = TANK_H_OX_M
        self.h_tank_f_m  = TANK_H_F_M
        self.g_force     = 1.0

    def get_current_thrust(self):
        st = self.state
        return self.mcc.get_thrust_kn(st[0], self.is_ignited, self.mdot_ox_last, self.mdot_f_last)

    @property
    def coolant_pressure_bar(self) -> float:
        w_f       = float(self.state[2])
        p_tank_f  = float(self.state[8])
        p_inlet_f = p_tank_f + ((RHO_LCH4_NOM * 9.81 * TANK_H_F_M) / 100000.0)
        return p_inlet_f + K_HEAD * (w_f**2)

    def system_equations(self, _t, y):
        (P_mcc, w_ox, w_f, th_mov, th_mfv, v_orhc, v_frhc,
         T_cool, P_tank_ox, P_tank_f, v_auto_ox, v_auto_f,
         P_orhc, P_frhc) = y

        P_mcc, P_orhc, P_frhc = [max(101325.0, p) for p in [P_mcc, P_orhc, P_frhc]]
        P_orhc_bar, P_frhc_bar = P_orhc/1e5, P_frhc/1e5

        dth_mov_dt    = self.valve_mov.get_derivative(self.cmd_th_mov, th_mov)
        dth_mfv_dt    = self.valve_mfv.get_derivative(self.cmd_th_mfv, th_mfv)
        dv_orhc_dt    = self.valve_orhc.get_derivative(self.cmd_v_orhc, v_orhc)
        dv_frhc_dt    = self.valve_frhc.get_derivative(self.cmd_v_frhc, v_frhc)
        dv_auto_ox_dt = self.valve_auto_ox.get_derivative(self.cmd_v_auto_ox, v_auto_ox)
        dv_auto_f_dt  = self.valve_auto_f.get_derivative(self.cmd_v_auto_f, v_auto_f)

        rho_ox = RHO_LOX
        rho_f  = CH4RealGasProps.density(self.t_tank_f, P_tank_f)
        p_inlet_ox = P_tank_ox + (rho_ox * 9.81 * self.g_force * self.h_tank_ox_m) / 1e5
        p_inlet_f  = P_tank_f  + (rho_f  * 9.81 * self.g_force * self.h_tank_f_m)  / 1e5

        P_dh_ox = p_inlet_ox + K_HEAD * (w_ox**2)
        P_dh_f  = p_inlet_f  + K_HEAD * (w_f**2)

        R_valve_ox = R_VALVE_OX_K / (max(0.0, th_mov)**3 + 1e-6)
        R_valve_f  = R_VALVE_F_K  / (max(0.0, th_mfv)**3 + 1e-6)

        rho_f_actual = CH4RealGasProps.density(T_cool, P_dh_f)
        R_jacket_dyn = self.R_jacket_base * np.sqrt(RHO_LCH4_NOM / max(rho_f_actual, 50.0))

        Cd2 = CD_INJ_LIQUID**2
        R_inj_ox_orhc = 1.0 / (2.0 * rho_ox      * Cd2 * A_INJ_OX_ORHC**2) / 1e5
        R_inj_f_frhc  = 1.0 / (2.0 * rho_f_actual * Cd2 * A_INJ_F_FRHC**2)  / 1e5

        # ── Lato ossidante ───────────────────────────────────────────────────
        ox_in_orhc = np.sqrt(max(0.0, P_dh_ox - P_orhc_bar) / (R_LINE_OX + R_valve_ox + R_inj_ox_orhc))
        P_man_ox = max(P_orhc_bar, P_dh_ox - (R_LINE_OX + R_valve_ox) * ox_in_orhc**2)
        ox_in_frhc = np.sqrt(max(0.0, P_dh_ox - P_frhc_bar) / R_BLEED_OX_FRHC) * v_frhc if self.is_ignited else 1e-6
        mdot_ox_tot = ox_in_orhc + ox_in_frhc

        # ── Lato combustibile ────────────────────────────────────────────────
        f_in_frhc = np.sqrt(max(0.0, P_dh_f - P_frhc_bar) / (R_LINE_F + R_valve_f + R_jacket_dyn + R_inj_f_frhc))
        P_man_f = max(P_frhc_bar, P_dh_f - (R_LINE_F + R_valve_f + R_jacket_dyn) * f_in_frhc**2)
        f_in_orhc = np.sqrt(max(0.0, P_dh_f - P_orhc_bar) / R_BLEED_F_ORHC) * v_orhc if self.is_ignited else 1e-6
        mdot_f_tot = f_in_frhc + f_in_orhc

        # Valvole sigillate: nessun flusso
        if th_mov < 0.01 and th_mfv < 0.01:
            ox_in_orhc = ox_in_frhc = mdot_ox_tot = 0.0
            f_in_frhc  = f_in_orhc  = mdot_f_tot  = 0.0
            P_man_ox = P_orhc_bar
            P_man_f  = P_frhc_bar

        ox_in_orhc = max(1e-6, ox_in_orhc)
        f_in_frhc  = max(1e-6, f_in_frhc)

        R_inj_ox_frhc = 1.0 / (2.0 * rho_ox      * Cd2 * A_INJ_OX_FRHC**2) / 1e5
        R_inj_f_orhc  = 1.0 / (2.0 * rho_f_actual * Cd2 * A_INJ_F_ORHC**2)  / 1e5

        cs_eff_orhc = self.orhc_chamber.get_c_star_eff(ox_in_orhc, f_in_orhc, P_orhc)
        cs_eff_frhc = self.frhc_chamber.get_c_star_eff(ox_in_frhc, f_in_frhc, P_frhc, t_fuel=T_cool)

        # ── Blocco 4: Termodinamica pre-burner, iniettori, turbine ───────────
        of_orhc = ox_in_orhc / max(f_in_orhc, 1e-9)
        of_frhc = ox_in_frhc / max(f_in_frhc, 1e-9)
        # ORHC: LOX non passa dal jacket → T_fuel_ref (default)
        # FRHC: CH4 entra dal jacket a T_cool → passa temperatura reale
        t_orhc = CEA_MethaloxCombustion.get_t_ad(of_orhc, P_orhc_bar) if self.is_ignited else 300.0
        t_frhc = CEA_MethaloxCombustion.get_t_ad(of_frhc, P_frhc_bar, t_fuel=T_cool) if self.is_ignited else 300.0

        gamma_o  = CEA_MethaloxCombustion.get_gamma(of_orhc)
        gamma_f  = CEA_MethaloxCombustion.get_gamma(of_frhc, t_fuel=T_cool)
        mw_o     = CEA_MethaloxCombustion.get_mw(of_orhc)
        mw_f     = CEA_MethaloxCombustion.get_mw(of_frhc, t_fuel=T_cool)
        cp_gas_o = (gamma_o / (gamma_o - 1.0)) * (8314.0 / mw_o)
        cp_gas_f = (gamma_f / (gamma_f - 1.0)) * (8314.0 / mw_f)

        p_back_ox_pa = max(P_orhc * P_BACK_FACTOR, P_mcc)
        p_back_f_pa  = max(P_frhc * P_BACK_FACTOR, P_mcc)
        mdot_out_orhc = self.orhc_chamber.get_exhaust_mass_flow(P_orhc, self.is_ignited, cs_eff_orhc,
                                                                  p_back_pa=p_back_ox_pa)
        mdot_out_frhc = self.frhc_chamber.get_exhaust_mass_flow(P_frhc, self.is_ignited, cs_eff_frhc,
                                                                  p_back_pa=p_back_f_pa)

        # ── Iniettori MCC: Bernoulli (gas ideale, densità da P/RT) ──────────
        A_inj_ox = A_INJ_GAS_OX; A_inj_f = A_INJ_GAS_F; Cd_inj = CD_INJ_GAS
        R_gas_o  = 8314.0 / max(mw_o, 1.0)
        R_gas_f  = 8314.0 / max(mw_f, 1.0)
        rho_gas_o = P_orhc / (R_gas_o * max(t_orhc, 300.0))
        rho_gas_f = P_frhc / (R_gas_f * max(t_frhc, 300.0))
        dP_inj_ox_pa = (mdot_out_orhc / max(Cd_inj * A_inj_ox, 1e-9))**2 / (2.0 * max(rho_gas_o, 0.1))
        dP_inj_f_pa  = (mdot_out_frhc / max(Cd_inj * A_inj_f,  1e-9))**2 / (2.0 * max(rho_gas_f, 0.1))
        P_man_ox_bar = max(P_orhc_bar - dP_inj_ox_pa / 1e5, P_mcc / 1e5)
        P_man_f_bar  = max(P_frhc_bar - dP_inj_f_pa  / 1e5, P_mcc / 1e5)

        dP_orhc_dt = self.orhc_chamber.get_derivative(P_orhc, ox_in_orhc, f_in_orhc,
                                                       self.is_ignited, p_back_pa=P_man_ox_bar * 1e5)
        dP_frhc_dt = self.frhc_chamber.get_derivative(P_frhc, ox_in_frhc, f_in_frhc,
                                                       self.is_ignited, p_back_pa=P_man_f_bar * 1e5)

        starter_ox = (th_mov * STARTER_TORQUE) * np.exp(-P_orhc_bar / STARTER_TAU_BAR)
        starter_f  = (th_mfv * STARTER_TORQUE) * np.exp(-P_frhc_bar / STARTER_TAU_BAR)

        dw_ox_dt = self.tp_ox.get_derivative(
            w_ox, P_orhc_bar, P_orhc_bar, P_man_ox_bar,
            t_orhc, mdot_out_orhc,
            gamma_o, cp_gas_o, starter_ox, p_inlet_ox, self.t_tank_ox
        )
        dw_f_dt = self.tp_fuel.get_derivative(
            w_f, P_frhc_bar, P_frhc_bar, P_man_f_bar,
            t_frhc, mdot_out_frhc,
            gamma_f, cp_gas_f, starter_f, p_inlet_f, self.t_tank_f
        )

        dP_tank_ox_dt = (v_auto_ox * AUTOG_RATE - mdot_ox_tot * AUTOG_BLEED_OX) if self.is_ignited else 0.0
        dP_tank_f_dt  = (v_auto_f  * AUTOG_RATE - mdot_f_tot  * AUTOG_BLEED_F)  if self.is_ignited else 0.0

        dT_cool_dt = self.jacket.get_derivative(T_cool, P_mcc, mdot_f_tot, self.is_ignited, 300.0, 0.5, coolant_p_bar=P_dh_f)
        # OF in MCC = composizione elementare reale (mdot_ox_tot/mdot_f_tot), non rapporto scarichi pre-burner
        dP_dt      = self.mcc.get_derivative(P_mcc, mdot_ox_tot, mdot_f_tot, self.is_ignited)

        return np.array([dP_dt, dw_ox_dt, dw_f_dt, dth_mov_dt, dth_mfv_dt,
                         dv_orhc_dt, dv_frhc_dt, dT_cool_dt, dP_tank_ox_dt, dP_tank_f_dt,
                         dv_auto_ox_dt, dv_auto_f_dt, dP_orhc_dt, dP_frhc_dt])

    def _update_diagnostics(self, y):
        """Ricalcola tutti i valori diagnostici dallo stato y e scrive su self."""
        (P_mcc, w_ox, w_f, th_mov, th_mfv, v_orhc, v_frhc,
         T_cool, P_tank_ox, P_tank_f, v_auto_ox, v_auto_f,
         P_orhc, P_frhc) = y

        P_mcc, P_orhc, P_frhc = [max(101325.0, p) for p in [P_mcc, P_orhc, P_frhc]]
        P_orhc_bar, P_frhc_bar = P_orhc / 1e5, P_frhc / 1e5

        rho_ox = RHO_LOX
        rho_f  = CH4RealGasProps.density(self.t_tank_f, P_tank_f)
        p_inlet_ox = P_tank_ox + (rho_ox * 9.81 * self.g_force * self.h_tank_ox_m) / 1e5
        p_inlet_f  = P_tank_f  + (rho_f  * 9.81 * self.g_force * self.h_tank_f_m)  / 1e5

        P_dh_ox = p_inlet_ox + K_HEAD * (w_ox**2)
        P_dh_f  = p_inlet_f  + K_HEAD * (w_f**2)

        R_valve_ox = R_VALVE_OX_K / (max(0.0, th_mov)**3 + 1e-6)
        R_valve_f  = R_VALVE_F_K  / (max(0.0, th_mfv)**3 + 1e-6)

        rho_f_actual = CH4RealGasProps.density(T_cool, P_dh_f)
        R_jacket_dyn = self.R_jacket_base * np.sqrt(RHO_LCH4_NOM / max(rho_f_actual, 50.0))

        Cd2 = CD_INJ_LIQUID**2
        R_inj_ox_orhc = 1.0 / (2.0 * rho_ox      * Cd2 * A_INJ_OX_ORHC**2) / 1e5
        R_inj_f_frhc  = 1.0 / (2.0 * rho_f_actual * Cd2 * A_INJ_F_FRHC**2)  / 1e5
        R_inj_ox_frhc = 1.0 / (2.0 * rho_ox       * Cd2 * A_INJ_OX_FRHC**2) / 1e5
        R_inj_f_orhc  = 1.0 / (2.0 * rho_f_actual  * Cd2 * A_INJ_F_ORHC**2) / 1e5

        ox_in_orhc = np.sqrt(max(0.0, P_dh_ox - P_orhc_bar) / (R_LINE_OX + R_valve_ox + R_inj_ox_orhc))
        P_man_ox   = max(P_orhc_bar, P_dh_ox - (R_LINE_OX + R_valve_ox) * ox_in_orhc**2)
        ox_in_frhc = np.sqrt(max(0.0, P_dh_ox - P_frhc_bar) / R_BLEED_OX_FRHC) * v_frhc if self.is_ignited else 1e-6
        mdot_ox_tot = ox_in_orhc + ox_in_frhc

        f_in_frhc  = np.sqrt(max(0.0, P_dh_f - P_frhc_bar) / (R_LINE_F + R_valve_f + R_jacket_dyn + R_inj_f_frhc))
        P_man_f    = max(P_frhc_bar, P_dh_f - (R_LINE_F + R_valve_f + R_jacket_dyn) * f_in_frhc**2)
        f_in_orhc  = np.sqrt(max(0.0, P_dh_f - P_orhc_bar) / R_BLEED_F_ORHC) * v_orhc if self.is_ignited else 1e-6
        mdot_f_tot = f_in_frhc + f_in_orhc

        if th_mov < 0.01 and th_mfv < 0.01:
            ox_in_orhc = ox_in_frhc = mdot_ox_tot = 0.0
            f_in_frhc  = f_in_orhc  = mdot_f_tot  = 0.0

        ox_in_orhc = max(1e-6, ox_in_orhc)
        f_in_frhc  = max(1e-6, f_in_frhc)

        of_orhc = ox_in_orhc / max(f_in_orhc, 1e-9)
        of_frhc = ox_in_frhc / max(f_in_frhc, 1e-9)

        t_orhc = CEA_MethaloxCombustion.get_t_ad(of_orhc, P_orhc_bar) if self.is_ignited else 300.0
        t_frhc = CEA_MethaloxCombustion.get_t_ad(of_frhc, P_frhc_bar, t_fuel=T_cool) if self.is_ignited else 300.0

        cs_eff_orhc = self.orhc_chamber.get_c_star_eff(ox_in_orhc, f_in_orhc, P_orhc)
        cs_eff_frhc = self.frhc_chamber.get_c_star_eff(ox_in_frhc, f_in_frhc, P_frhc, t_fuel=T_cool)

        p_back_ox_pa = max(P_orhc * P_BACK_FACTOR, P_mcc)
        p_back_f_pa  = max(P_frhc * P_BACK_FACTOR, P_mcc)
        mdot_out_orhc = self.orhc_chamber.get_exhaust_mass_flow(P_orhc, self.is_ignited, cs_eff_orhc,
                                                                  p_back_pa=p_back_ox_pa)
        mdot_out_frhc = self.frhc_chamber.get_exhaust_mass_flow(P_frhc, self.is_ignited, cs_eff_frhc,
                                                                  p_back_pa=p_back_f_pa)

        mw_o    = CEA_MethaloxCombustion.get_mw(of_orhc)
        mw_f    = CEA_MethaloxCombustion.get_mw(of_frhc, t_fuel=T_cool)
        R_gas_o = 8314.0 / max(mw_o, 1.0)
        R_gas_f = 8314.0 / max(mw_f, 1.0)

        A_inj_ox = A_INJ_GAS_OX; A_inj_f = A_INJ_GAS_F; Cd_inj = CD_INJ_GAS
        rho_gas_o = P_orhc / (R_gas_o * max(t_orhc, 300.0))
        rho_gas_f = P_frhc / (R_gas_f * max(t_frhc, 300.0))
        dP_inj_ox_pa = (mdot_out_orhc / max(Cd_inj * A_inj_ox, 1e-9))**2 / (2.0 * max(rho_gas_o, 0.1))
        dP_inj_f_pa  = (mdot_out_frhc / max(Cd_inj * A_inj_f,  1e-9))**2 / (2.0 * max(rho_gas_f, 0.1))
        P_man_ox_bar = max(P_orhc_bar - dP_inj_ox_pa / 1e5, P_mcc / 1e5)
        P_man_f_bar  = max(P_frhc_bar - dP_inj_f_pa  / 1e5, P_mcc / 1e5)

        self.of_orhc_current    = of_orhc
        self.of_frhc_current    = of_frhc
        self.t_orhc_current     = t_orhc
        self.t_frhc_current     = t_frhc
        self.mdot_ox_last       = mdot_ox_tot
        self.mdot_f_last        = mdot_f_tot
        self.mdot_out_orhc_last = mdot_out_orhc
        self.mdot_out_frhc_last = mdot_out_frhc
        self.dp_inj_ox_bar      = dP_inj_ox_pa / 1e5
        self.dp_inj_f_bar       = dP_inj_f_pa  / 1e5
        self.dp_inj_ox_orhc_bar = R_inj_ox_orhc * ox_in_orhc**2
        self.dp_inj_f_frhc_bar  = R_inj_f_frhc  * f_in_frhc**2
        self.dp_inj_ox_frhc_bar = R_inj_ox_frhc * ox_in_frhc**2
        self.dp_inj_f_orhc_bar  = R_inj_f_orhc  * f_in_orhc**2
        self.p_man_ox_bar       = P_man_ox
        self.p_man_f_bar        = P_man_f

    def step(self, dt):
        t0 = self.current_time
        t1 = t0 + dt
        sol = solve_ivp(
            self.system_equations, [t0, t1], self.state,
            method='BDF', rtol=1e-3, atol=1e-4, max_step=dt
        )
        if sol.success:
            self.state = sol.y[:, -1]
        else:
            # Fallback RK4 se BDF fallisce
            y  = self.state
            k1 = self.system_equations(0,        y)
            k2 = self.system_equations(0.5 * dt, y + 0.5 * dt * k1)
            k3 = self.system_equations(0.5 * dt, y + 0.5 * dt * k2)
            k4 = self.system_equations(dt,       y + dt * k3)
            self.state = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        # RPM non può essere negativo fisicamente
        self.state[1] = max(0.0, self.state[1])  # w_ox
        self.state[2] = max(0.0, self.state[2])  # w_f
        self._update_diagnostics(self.state)
        self.current_time += dt