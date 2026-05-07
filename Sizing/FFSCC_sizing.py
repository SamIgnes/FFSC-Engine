"""
FFSCC_sizing.py — Design-point sizing for CH4/LOX Full-Flow Staged Combustion Cycle
======================================================================================
Design inputs (bottom of file):
  F_kN       = 2750 kN  (vacuum thrust)
  Pc_bar     = 300 bar  (MCC chamber pressure)
  OF_mcc     = 3.4      (main chamber mixture ratio)
  OF_orhc    = 50       (ox-rich preburner, ORHC)
  OF_frhc    = 0.2      (fuel-rich preburner, FRHC)
  eps        = 40       (nozzle expansion ratio)

Outputs:
  1. System mass-flow budget
  2. Turbopump power balance
  3. Injector areas
  4. Hydraulic resistances
  5. Ready-to-paste config.py block
"""

import math
import numpy as np

# ── Thermochemical data (from CEA, CH4/LOX) ──────────────────────────────────
# Hardcoded lookup: (OF, Pc_bar) → (c* m/s, T_ad K, gamma, cp J/kg/K, Isp_vac s)
# Sources: CEA-Web runs at relevant conditions
# MCC: OF=3.4, Pc=300 bar
_MCC_CSTAR      = 1858.0   # m/s
_MCC_TAD        = 3560.0   # K
_MCC_GAMMA      = 1.138
_MCC_ISP_VAC    = 380.0    # s   (vacuum, eps=40)

# ORHC preburner: OF=50 (ox-rich), Pc≈400 bar
# Almost pure O2 with trace CH4 — cold
_ORHC_TAD       = 810.0    # K   (CEA OF=50)
_ORHC_GAMMA     = 1.35
_ORHC_CP        = 1020.0   # J/kg/K  (mostly O2)
_ORHC_CSTAR     = 1050.0   # m/s

# FRHC preburner: OF=0.2 (fuel-rich), Pc≈400 bar
# Almost pure CH4 with trace LOX — cold
_FRHC_TAD       = 1050.0   # K   (CEA OF=0.2)
_FRHC_GAMMA     = 1.20
_FRHC_CP        = 3200.0   # J/kg/K  (mostly CH4, high cp)
_FRHC_CSTAR     = 900.0    # m/s

# ── Propellant properties ─────────────────────────────────────────────────────
RHO_LOX    = 1141.0   # kg/m³  (90 K, ~6 bar)
RHO_LCH4   = 422.0    # kg/m³  (110 K, ~6 bar)
G0         = 9.80665  # m/s²

# ── Design assumptions ────────────────────────────────────────────────────────
ETA_PUMP        = 0.75   # pump isentropic efficiency
ETA_TURBINE     = 0.70   # turbine isentropic efficiency
P_PUMP_OUT_BAR  = 720.0  # pump outlet pressure [bar]  (P_preburner + injector ΔP ~50 bar)
P_TANK_BAR      = 6.5    # tank pressure [bar]
P_PREBURNER_BAR = 670.0  # preburner chamber pressure [bar]  (≈2.2× Pc_MCC, Raptor-like)
P_MCC_BAR       = 300.0  # MCC pressure [bar]

CD_LIQ = 0.92   # discharge coeff, liquid injectors
CD_GAS = 0.72   # discharge coeff, gas injectors

# Turbopump design: target tip-speed head coefficient
PSI_PUMP    = 0.50   # head coefficient ψ = ΔH/(u²/g)
RPM_OX_DES  = 22000  # [RPM] target LOX pump speed
RPM_F_DES   = 30000  # [RPM] target CH4 pump speed


# ═══════════════════════════════════════════════════════════════════════════════
def run_sizing(F_kN, Pc_bar, OF_mcc, OF_orhc, OF_frhc, eps):
    print("=" * 70)
    print("FFSCC DESIGN-POINT SIZING  —  CH4/LOX")
    print(f"  F={F_kN} kN | Pc={Pc_bar} bar | OF_mcc={OF_mcc}")
    print(f"  OF_orhc={OF_orhc} | OF_frhc={OF_frhc} | eps={eps}")
    print("=" * 70)

    # ── 1. MASS FLOW BUDGET ───────────────────────────────────────────────────
    F_N        = F_kN * 1e3
    mdot_total = F_N / (_MCC_ISP_VAC * G0)
    mdot_ox    = mdot_total * OF_mcc / (1 + OF_mcc)
    mdot_f     = mdot_total * 1.0    / (1 + OF_mcc)

    # FFSCC flow split across preburners
    # Variables: x = mdot_f in ORHC, y = mdot_f in FRHC
    # x + y = mdot_f
    # OF_orhc * x + OF_frhc * y = mdot_ox   (ox conservation)
    # → x*(OF_orhc - OF_frhc) = mdot_ox - OF_frhc*mdot_f
    x = (mdot_ox - OF_frhc * mdot_f) / (OF_orhc - OF_frhc)
    y = mdot_f - x
    mdot_f_orhc  = x     # CH4 entering ORHC
    mdot_f_frhc  = y     # CH4 entering FRHC
    mdot_ox_orhc = OF_orhc * x   # LOX entering ORHC
    mdot_ox_frhc = OF_frhc * y   # LOX entering FRHC

    mdot_ORHC = mdot_ox_orhc + mdot_f_orhc
    mdot_FRHC = mdot_ox_frhc + mdot_f_frhc

    print("\n── 1. MASS FLOW BUDGET ──────────────────────────────────────────────")
    print(f"  mdot_total  = {mdot_total:.2f} kg/s")
    print(f"  mdot_ox     = {mdot_ox:.2f} kg/s  |  mdot_f = {mdot_f:.2f} kg/s")
    print(f"  ORHC:  {mdot_ox_orhc:.1f} kg/s LOX + {mdot_f_orhc:.1f} kg/s CH4 = {mdot_ORHC:.1f} kg/s")
    print(f"  FRHC:  {mdot_ox_frhc:.1f} kg/s LOX + {mdot_f_frhc:.1f} kg/s CH4 = {mdot_FRHC:.1f} kg/s")
    print(f"  Check  ORHC+FRHC = {mdot_ORHC+mdot_FRHC:.1f} kg/s  (total = {mdot_total:.1f})")

    # ── 2. NOZZLE & THROAT ───────────────────────────────────────────────────
    Pc_pa      = Pc_bar * 1e5
    A_throat   = mdot_total * _MCC_CSTAR / Pc_pa
    r_throat   = math.sqrt(A_throat / math.pi)
    A_exit     = eps * A_throat
    r_exit     = math.sqrt(A_exit / math.pi)

    # Preburner throats: sized to choke at P_preburner
    A_throat_orhc = mdot_ORHC * _ORHC_CSTAR / (P_PREBURNER_BAR * 1e5)
    A_throat_frhc = mdot_FRHC * _FRHC_CSTAR / (P_PREBURNER_BAR * 1e5)

    print("\n── 2. NOZZLE & CHAMBER GEOMETRY ─────────────────────────────────────")
    print(f"  MCC A_throat   = {A_throat:.5f} m²  (r = {r_throat*100:.2f} cm)")
    print(f"  MCC A_exit     = {A_exit:.4f} m²    (r = {r_exit*100:.2f} cm)")
    print(f"  ORHC A_throat  = {A_throat_orhc:.5f} m²")
    print(f"  FRHC A_throat  = {A_throat_frhc:.5f} m²")

    # Chamber volumes: L* = 1.5 m (MCC), 0.8 m (preburners)
    L_star_mcc  = 1.5  # m  characteristic length
    L_star_pb   = 0.8  # m
    V_mcc   = L_star_mcc * A_throat
    V_orhc  = L_star_pb  * A_throat_orhc
    V_frhc  = L_star_pb  * A_throat_frhc
    print(f"  MCC volume     = {V_mcc:.4f} m³  (L*={L_star_mcc} m)")
    print(f"  ORHC volume    = {V_orhc:.5f} m³")
    print(f"  FRHC volume    = {V_frhc:.5f} m³")

    # ── 3. PUMP SIZING ───────────────────────────────────────────────────────
    dP_ox  = (P_PUMP_OUT_BAR - P_TANK_BAR) * 1e5  # Pa
    dP_f   = (P_PUMP_OUT_BAR - P_TANK_BAR) * 1e5

    W_pump_ox = mdot_ox * dP_ox / (RHO_LOX  * ETA_PUMP)   # W
    W_pump_f  = mdot_f  * dP_f  / (RHO_LCH4 * ETA_PUMP)

    # Head at design RPM → K_HEAD_ox, K_HEAD_f
    # ΔP [bar] = K_HEAD * RPM²  →  K_HEAD = ΔP_bar / RPM²
    dP_bar     = P_PUMP_OUT_BAR - P_TANK_BAR
    K_HEAD_ox  = dP_bar / RPM_OX_DES**2
    K_HEAD_f   = dP_bar / RPM_F_DES**2

    # Impeller tip speed & diameter from ψ = g*H/u²
    H_ox = dP_ox / (RHO_LOX  * G0)  # m
    H_f  = dP_f  / (RHO_LCH4 * G0)
    u2_ox = math.sqrt(G0 * H_ox / PSI_PUMP)
    u2_f  = math.sqrt(G0 * H_f  / PSI_PUMP)
    omega_ox = RPM_OX_DES * math.pi / 30
    omega_f  = RPM_F_DES  * math.pi / 30
    D2_ox = 2.0 * u2_ox / omega_ox
    D2_f  = 2.0 * u2_f  / omega_f

    # Moment of inertia estimate: solid disk I = 0.5 * m * r²
    # Turbopump rotor mass estimate from tip speed and specific speed
    m_rotor_ox = 60.0   # kg  rough estimate for this thrust class
    m_rotor_f  = 35.0
    I_ox = 0.5 * m_rotor_ox * (D2_ox/2)**2
    I_f  = 0.5 * m_rotor_f  * (D2_f /2)**2
    # Convert from rad basis to RPM basis: I_rpm = I_rad * (pi/30)²  → already handled in engine.py
    # engine.py uses: d(RPM)/dt = torque / I_rpm  where I_rpm = I_rad * (pi/30)^2 ... check
    # Standard: I_rpm = I_kg_m2 * (30/pi)  [kg·m²·RPM/s... messy units]
    # engine.py config: TP_OX_INERTIA = 0.9 * 30/pi ≈ 8.6 — that IS I_rad * 30/pi for RPM equations
    I_ox_cfg = I_ox * (30 / math.pi)
    I_f_cfg  = I_f  * (30 / math.pi)

    print("\n── 3. TURBOPUMP PUMP SIZING ──────────────────────────────────────────")
    print(f"  LOX pump:  W={W_pump_ox/1e6:.2f} MW | ΔP={dP_bar:.0f} bar | D2={D2_ox*100:.1f} cm | RPM_des={RPM_OX_DES}")
    print(f"  CH4 pump:  W={W_pump_f/1e6:.2f} MW | ΔP={dP_bar:.0f} bar | D2={D2_f*100:.1f} cm  | RPM_des={RPM_F_DES}")
    print(f"  K_HEAD_ox  = {K_HEAD_ox:.4e} bar/RPM²")
    print(f"  K_HEAD_f   = {K_HEAD_f:.4e} bar/RPM²")
    print(f"  I_ox (cfg) = {I_ox_cfg:.3f}  |  I_f (cfg) = {I_f_cfg:.3f}")

    # ── 4. TURBINE POWER BALANCE ─────────────────────────────────────────────
    # ORHC turbine drives FUEL pump
    # FRHC turbine drives OX pump
    PR_orhc = P_PREBURNER_BAR / P_MCC_BAR  # expansion ratio across turbine
    PR_frhc = P_PREBURNER_BAR / P_MCC_BAR

    k_orhc = _ORHC_GAMMA
    k_frhc = _FRHC_GAMMA
    dH_orhc = (_ORHC_CP * _ORHC_TAD * ETA_TURBINE *
               (1.0 - PR_orhc**(-((k_orhc-1)/k_orhc))))
    dH_frhc = (_FRHC_CP * _FRHC_TAD * ETA_TURBINE *
               (1.0 - PR_frhc**(-((k_frhc-1)/k_frhc))))

    W_turb_orhc = dH_orhc * mdot_ORHC   # drives fuel pump
    W_turb_frhc = dH_frhc * mdot_FRHC   # drives ox pump

    print("\n── 4. TURBINE POWER BALANCE ──────────────────────────────────────────")
    print(f"  Turbine expansion ratio: {PR_orhc:.2f}  ({P_PREBURNER_BAR:.0f} → {P_MCC_BAR:.0f} bar)")
    print(f"  ORHC turbine (→ CH4 pump):")
    print(f"    W_turb = {W_turb_orhc/1e6:.2f} MW  |  W_pump_f = {W_pump_f/1e6:.2f} MW  |  margin = {(W_turb_orhc-W_pump_f)/1e6:+.2f} MW")
    print(f"  FRHC turbine (→ LOX pump):")
    print(f"    W_turb = {W_turb_frhc/1e6:.2f} MW  |  W_pump_ox= {W_pump_ox/1e6:.2f} MW  |  margin = {(W_turb_frhc-W_pump_ox)/1e6:+.2f} MW")
    if W_turb_orhc < W_pump_f or W_turb_frhc < W_pump_ox:
        print("  *** WARNING: turbine underpowered — raise Pc_preburner or T_ad ***")

    # ── 5. PUMP TORQUE COEFFICIENTS ──────────────────────────────────────────
    # In engine.py: tau_pump_drag ≈ K1*RPM² + K2*RPM*P_outlet
    # At design: tau_pump * omega = W_pump
    # tau_ox_des = W_pump_ox / omega_ox
    # Simplified: K1 * RPM² = 0.7 * tau,  K2 * RPM * P_out = 0.3 * tau
    tau_ox = W_pump_ox / omega_ox
    tau_f  = W_pump_f  / omega_f
    # K2: engine.py uses tau_p = K1*RPM² + K2*RPM*p_out_pump_bar  (p in bar, not Pa)
    K1_ox = 0.7 * tau_ox / RPM_OX_DES**2
    K2_ox = 0.3 * tau_ox / (RPM_OX_DES * P_PUMP_OUT_BAR)
    K1_f  = 0.7 * tau_f  / RPM_F_DES**2
    K2_f  = 0.3 * tau_f  / (RPM_F_DES  * P_PUMP_OUT_BAR)

    print("\n── 5. PUMP TORQUE COEFFICIENTS ──────────────────────────────────────")
    print(f"  τ_ox_des = {tau_ox:.1f} N·m  |  τ_f_des = {tau_f:.1f} N·m")
    print(f"  K1_ox = {K1_ox:.5e}  K2_ox = {K2_ox:.5e}")
    print(f"  K1_f  = {K1_f:.5e}  K2_f  = {K2_f:.5e}")

    # ── 6. NPSH (cavitation limit) ────────────────────────────────────────────
    # NPSH_required = Kn * RPM²
    # Kn from Thomas correlation: σ_c = 0.02..0.10, NPSH_r = σ_c * H_total
    # subsystems.py: npsh_avail = p_inlet_bar - p_vapor_bar
    #                npsh_req   = npsh_coeff * RPM²   → coeff in [bar/RPM²]
    sigma_c = 0.04
    NPSH_ox_m  = sigma_c * H_ox                                    # m
    NPSH_f_m   = sigma_c * H_f
    NPSH_ox_bar = NPSH_ox_m * RHO_LOX  * G0 / 1e5                 # bar
    NPSH_f_bar  = NPSH_f_m  * RHO_LCH4 * G0 / 1e5
    Kn_ox = NPSH_ox_bar / RPM_OX_DES**2                           # bar/RPM²
    Kn_f  = NPSH_f_bar  / RPM_F_DES**2
    print("\n── 6. NPSH COEFFICIENTS ─────────────────────────────────────────────")
    print(f"  NPSH_ox_req = {NPSH_ox_m:.1f} m = {NPSH_ox_bar:.2f} bar  →  Kn_ox = {Kn_ox:.4e} bar/RPM²")
    print(f"  NPSH_f_req  = {NPSH_f_m:.1f} m = {NPSH_f_bar:.2f} bar  →  Kn_f  = {Kn_f:.4e} bar/RPM²")

    # ── 7. INJECTOR AREAS (Bernoulli) ─────────────────────────────────────────
    # ΔP across injectors: pump_out - preburner
    dP_inj_liq = (P_PUMP_OUT_BAR - P_PREBURNER_BAR) * 1e5  # Pa

    # ORHC: LOX main injector
    A_inj_ox_orhc = mdot_ox_orhc / (CD_LIQ * math.sqrt(2 * RHO_LOX  * dP_inj_liq))
    # FRHC: CH4 main injector
    A_inj_f_frhc  = mdot_f_frhc  / (CD_LIQ * math.sqrt(2 * RHO_LCH4 * dP_inj_liq))
    # Cross-bleed: CH4 → ORHC
    A_inj_f_orhc  = mdot_f_orhc  / (CD_LIQ * math.sqrt(2 * RHO_LCH4 * dP_inj_liq))
    # Cross-bleed: LOX → FRHC
    A_inj_ox_frhc = mdot_ox_frhc / (CD_LIQ * math.sqrt(2 * RHO_LOX  * dP_inj_liq))

    # MCC gas injectors: ORHC-gas & FRHC-gas
    # Gas density at injection: ideal gas approx
    R_O2  = 8314.0 / 32.0   # J/kg/K
    R_CH4 = 8314.0 / 16.0
    T_orhc_exit = _ORHC_TAD - dH_orhc / _ORHC_CP
    T_frhc_exit = _FRHC_TAD - dH_frhc / _FRHC_CP
    rho_gas_ox  = P_MCC_BAR * 1.10 * 1e5 / (R_O2  * max(T_orhc_exit, 200.0))
    rho_gas_f   = P_MCC_BAR * 1.10 * 1e5 / (R_CH4 * max(T_frhc_exit, 200.0))
    dP_inj_gas  = (P_PREBURNER_BAR - P_MCC_BAR) * 0.15 * 1e5  # ~15% of ΔP for gas inj
    A_inj_gas_ox = mdot_ORHC / (CD_GAS * math.sqrt(max(2 * rho_gas_ox * dP_inj_gas, 1.0)))
    A_inj_gas_f  = mdot_FRHC / (CD_GAS * math.sqrt(max(2 * rho_gas_f  * dP_inj_gas, 1.0)))

    print("\n── 7. INJECTOR AREAS ────────────────────────────────────────────────")
    print(f"  ΔP_inj_liquid = {dP_inj_liq/1e5:.0f} bar")
    print(f"  A_INJ_OX_ORHC = {A_inj_ox_orhc:.5f} m²  (LOX → ORHC, {mdot_ox_orhc:.0f} kg/s)")
    print(f"  A_INJ_F_FRHC  = {A_inj_f_frhc:.5f} m²  (CH4 → FRHC, {mdot_f_frhc:.0f} kg/s)")
    print(f"  A_INJ_F_ORHC  = {A_inj_f_orhc:.6f} m²  (CH4 cross → ORHC, {mdot_f_orhc:.1f} kg/s)")
    print(f"  A_INJ_OX_FRHC = {A_inj_ox_frhc:.6f} m²  (LOX cross → FRHC, {mdot_ox_frhc:.1f} kg/s)")
    print(f"  Gas ORHC exit: T={T_orhc_exit:.0f} K  rho={rho_gas_ox:.1f} kg/m³")
    print(f"  Gas FRHC exit: T={T_frhc_exit:.0f} K  rho={rho_gas_f:.1f} kg/m³")
    print(f"  ΔP_inj_gas    = {dP_inj_gas/1e5:.1f} bar")
    print(f"  A_INJ_GAS_OX  = {A_inj_gas_ox:.5f} m²  (ORHC gas → MCC, {mdot_ORHC:.0f} kg/s)")
    print(f"  A_INJ_GAS_F   = {A_inj_gas_f:.5f} m²   (FRHC gas → MCC, {mdot_FRHC:.0f} kg/s)")

    # ── 8. HYDRAULIC RESISTANCES ──────────────────────────────────────────────
    # R = ΔP [bar] / mdot² [kg/s]²
    # Line LOX: ΔP_line ≈ 5 bar at full flow
    dP_line = 5.0  # bar
    R_line_ox = dP_line / mdot_ox**2
    R_line_f  = dP_line / mdot_f**2

    # Valve resistance: at full open (th=1) with design mdot
    # ΔP_valve ≈ R_valve * mdot² / th³
    # Set so that at th=0.8, ΔP_valve ≈ 10 bar
    th_nom = 0.8
    dP_valve = 10.0  # bar target at th_nom
    R_valve_ox = dP_valve * th_nom**3 / mdot_ox**2
    R_valve_f  = dP_valve * th_nom**3 / mdot_f**2

    # Jacket resistance: ΔP ≈ 20 bar at nominal coolant flow
    mdot_cool_nom = mdot_f * 0.95  # most CH4 cools jacket
    dP_jacket = 20.0  # bar
    R_jacket = dP_jacket / mdot_cool_nom**2

    # Cross-bleed resistances
    # R_bleed = ΔP_bleed / mdot_bleed²
    dP_bleed = (P_PUMP_OUT_BAR - P_PREBURNER_BAR)  # bar drop across bleed line
    R_bleed_f_orhc = dP_bleed / mdot_f_orhc**2     # CH4 → ORHC
    R_bleed_ox_frhc = dP_bleed / mdot_ox_frhc**2   # LOX → FRHC

    print("\n── 8. HYDRAULIC RESISTANCES ──────────────────────────────────────────")
    print(f"  R_LINE_OX      = {R_line_ox:.6f} bar/(kg/s)²")
    print(f"  R_LINE_F       = {R_line_f:.6f} bar/(kg/s)²")
    print(f"  R_VALVE_OX_K   = {R_valve_ox:.6f} bar/(kg/s)²")
    print(f"  R_VALVE_F_K    = {R_valve_f:.6f} bar/(kg/s)²")
    print(f"  R_JACKET_BASE  = {R_jacket:.6f} bar/(kg/s)²")
    print(f"  R_BLEED_F_ORHC = {R_bleed_f_orhc:.4f} bar/(kg/s)²")
    print(f"  R_BLEED_OX_FRHC= {R_bleed_ox_frhc:.4f} bar/(kg/s)²")

    # ── 9. COOLING JACKET THERMAL ─────────────────────────────────────────────
    # Nominal coolant mass flow through jacket = fuel flow
    JACKET_MDOT_NOM = mdot_cool_nom
    print("\n── 9. COOLING JACKET ────────────────────────────────────────────────")
    print(f"  JACKET_MDOT_NOM = {JACKET_MDOT_NOM:.1f} kg/s  (~95% of CH4)")

    # ── 10. CONFIG.PY OUTPUT BLOCK ────────────────────────────────────────────
    # Use average K_HEAD (single value in current model, LOX-biased since it's heavier load)
    K_HEAD_avg = K_HEAD_ox  # use LOX pump (dominant)

    print("\n" + "=" * 70)
    print("CONFIG.PY  —  REPLACE BLOCK  (paste into config.py)")
    print("=" * 70)
    print(f"""
# ── Sizing-derived constants  [FFSCC_sizing.py  F={F_kN}kN Pc={Pc_bar}bar OF={OF_mcc}] ──

# Design-point propellant flows
# mdot_total = {mdot_total:.1f} kg/s  |  mdot_ox = {mdot_ox:.1f} kg/s  |  mdot_f = {mdot_f:.1f} kg/s

# ── Turbopompe ────────────────────────────────────────────────────────────────
# Testa pompa: P_dh = p_inlet + K_HEAD_ox * RPM²  [bar]
K_HEAD_OX = {K_HEAD_ox:.4e}   # [bar/RPM²] LOX pump  (RPM_des={RPM_OX_DES})
K_HEAD_F  = {K_HEAD_f:.4e}   # [bar/RPM²] CH4 pump  (RPM_des={RPM_F_DES})
# NOTE: current engine.py uses single K_HEAD — recommend K_HEAD = K_HEAD_OX

K_HEAD = {K_HEAD_ox:.4e}   # [bar/RPM²] — use LOX value (conservative)

TP_OX_INERTIA   = {I_ox_cfg:.4f}   # [kg·m²·30/π] inertia LOX turbopump
TP_OX_PUMP_K1   = {K1_ox:.6f}   # [N·m/RPM²] drag quadratic
TP_OX_PUMP_K2   = {K2_ox:.4e}   # [N·m/(RPM·Pa)] drag linear
TP_OX_NPSH      = {Kn_ox:.4e}   # [m/RPM²] NPSH coeff LOX

TP_F_INERTIA    = {I_f_cfg:.4f}   # [kg·m²·30/π] inertia CH4 turbopump
TP_F_PUMP_K1    = {K1_f:.6f}   # [N·m/RPM²] drag quadratic
TP_F_PUMP_K2    = {K2_f:.4e}   # [N·m/(RPM·Pa)] drag linear
TP_F_NPSH       = {Kn_f:.4e}   # [m/RPM²] NPSH coeff CH4

TP_ETA_TURBINE  = {ETA_TURBINE}
TP_OVERSPEED_RPM = {max(RPM_OX_DES, RPM_F_DES) * 1.3:.0f}

# ── Valvole ───────────────────────────────────────────────────────────────────
# Resistenze idrauliche (modello pressione: ΔP [bar] = R * mdot² / th^3 per linee principali)
R_VALVE_OX_K    = {R_valve_ox:.6f}   # [bar/(kg/s)²]
R_VALVE_F_K     = {R_valve_f:.6f}   # [bar/(kg/s)²]
R_LINE_OX       = {R_line_ox:.6f}   # [bar/(kg/s)²]
R_LINE_F        = {R_line_f:.6f}   # [bar/(kg/s)²]
R_JACKET_BASE   = {R_jacket:.6f}   # [bar/(kg/s)²]
R_BLEED_OX_FRHC = {R_bleed_ox_frhc:.4f}   # [bar/(kg/s)²] LOX→FRHC cross-bleed
R_BLEED_F_ORHC  = {R_bleed_f_orhc:.4f}   # [bar/(kg/s)²] CH4→ORHC cross-bleed

# ── Iniettori ─────────────────────────────────────────────────────────────────
A_INJ_OX_ORHC   = {A_inj_ox_orhc:.5f}   # [m²] LOX → ORHC
A_INJ_F_FRHC    = {A_inj_f_frhc:.5f}   # [m²] CH4 → FRHC
A_INJ_OX_FRHC   = {A_inj_ox_frhc:.6f}  # [m²] LOX cross-bleed → FRHC
A_INJ_F_ORHC    = {A_inj_f_orhc:.6f}  # [m²] CH4 cross-bleed → ORHC
A_INJ_GAS_OX    = {A_inj_gas_ox:.5f}   # [m²] ORHC gas → MCC
A_INJ_GAS_F     = {A_inj_gas_f:.5f}   # [m²] FRHC gas → MCC

# ── Camere di combustione ─────────────────────────────────────────────────────
ORHC_VOLUME     = {V_orhc:.5f}   # [m³]
ORHC_A_THROAT   = {A_throat_orhc:.5f}   # [m²]
ORHC_ETA_CSTAR  = 0.98

FRHC_VOLUME     = {V_frhc:.5f}   # [m³]
FRHC_A_THROAT   = {A_throat_frhc:.5f}   # [m²]
FRHC_ETA_CSTAR  = 0.98

MCC_VOLUME      = {V_mcc:.5f}   # [m³]
MCC_A_THROAT    = {A_throat:.5f}   # [m²]
MCC_EPS_NOZZLE  = {eps:.1f}
MCC_ETA_CSTAR   = 0.97

# ── Jacket ────────────────────────────────────────────────────────────────────
JACKET_MDOT_NOM = {JACKET_MDOT_NOM:.1f}   # [kg/s] nominal coolant (CH4)

# ── Target operativi ──────────────────────────────────────────────────────────
TARGET_OF_MCC   = {OF_mcc}
TARGET_OF_ORHC  = {OF_orhc:.1f}
TARGET_OF_FRHC  = {OF_frhc}
SIM_TARGET_KN   = {F_kN}
""")

    print("=" * 70)
    print("DESIGN SUMMARY")
    print("=" * 70)
    print(f"  Thrust (vac)  : {F_kN} kN")
    print(f"  Isp (vac)     : {_MCC_ISP_VAC} s")
    print(f"  Pc (MCC)      : {Pc_bar} bar")
    print(f"  OF_mcc        : {OF_mcc}")
    print(f"  mdot_total    : {mdot_total:.1f} kg/s")
    print(f"  A_throat_mcc  : {A_throat*1e4:.1f} cm²  ({r_throat*100:.2f} cm radius)")
    print(f"  LOX pump      : {W_pump_ox/1e6:.1f} MW @ {RPM_OX_DES} RPM,  D={D2_ox*100:.1f} cm")
    print(f"  CH4 pump      : {W_pump_f/1e6:.1f} MW @ {RPM_F_DES} RPM,  D={D2_f*100:.1f} cm")
    print(f"  ORHC flow     : {mdot_ORHC:.1f} kg/s  (T_ad={_ORHC_TAD} K, turb_out={T_orhc_exit:.0f} K)")
    print(f"  FRHC flow     : {mdot_FRHC:.1f} kg/s  (T_ad={_FRHC_TAD} K, turb_out={T_frhc_exit:.0f} K)")
    print("=" * 70)


if __name__ == "__main__":
    run_sizing(
        F_kN    = 2750.0,
        Pc_bar  = 300.0,
        OF_mcc  = 3.4,
        OF_orhc = 50.0,
        OF_frhc = 0.2,
        eps     = 40.0,
    )
