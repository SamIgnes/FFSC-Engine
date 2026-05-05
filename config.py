"""
config.py — Parametri fisici centralizzati FFSCC CH4/LOX

Modifica qui per cambiare qualsiasi costante del motore.
Tutti gli altri moduli importano da questo file.
"""

# ── Simulazione ───────────────────────────────────────────────────────────────
SIM_DT         = 0.05    # [s] passo temporale
SIM_T_TOTAL    = 50.0    # [s] durata simulazione headless
SIM_START_SEQ  = 5.0     # [s] tempo avvio sequenza accensione
SIM_TARGET_KN  = 2750.0  # [kN] spinta target

# ── Propellanti: geometria serbatoi ──────────────────────────────────────────
TANK_P_INIT_BAR   = 6.5    # [bar] pressione iniziale serbatoi (entrambi)
TANK_T_OX_INIT    = 80.0   # [K]   temperatura iniziale LOX
TANK_T_F_INIT     = 100.0  # [K]   temperatura iniziale LCH4
TANK_H_OX_M       = 20.0   # [m]   altezza colonna LOX (pressione idrostatica)
TANK_H_F_M        = 25.0   # [m]   altezza colonna LCH4
RHO_LOX           = 1141.0 # [kg/m³] densità LOX (costante, fase liquida)
RHO_LCH4_NOM     = 422.0  # [kg/m³] densità LCH4 nominale (fallback)

# ── Turbopompe ────────────────────────────────────────────────────────────────
# Testa pompa: P_dh = p_inlet + K_HEAD * RPM²  [bar]
K_HEAD = 10.5e-6   # [bar/RPM²] — identico per LOX e LCH4 (pompe bilanciate)

# Turbopompa LOX
TP_OX_INERTIA   = 0.9 * 30.0 / 3.14159265  # [kg·m²] inerzia (scalata rad→RPM)
TP_OX_PUMP_K1   = 0.00005                   # coppia quadratica [N·m/RPM²]
TP_OX_PUMP_K2   = 0.002                     # coppia lineare    [N·m/(RPM·bar)]
TP_OX_NPSH      = 0.7e-8                    # coefficiente NPSH [RPM⁻²]

# Turbopompa LCH4
TP_F_INERTIA    = 0.5 * 30.0 / 3.14159265
TP_F_PUMP_K1    = 0.000006
TP_F_PUMP_K2    = 0.0009
TP_F_NPSH       = 0.25e-8

# Efficienza isentropica turbina
TP_ETA_TURBINE  = 0.70

# Freno overspeed (RPM limite design)
TP_OVERSPEED_RPM  = 38000.0
TP_OVERSPEED_K    = 100.0   # [N·m/RPM²] penalità quadratica oltre il limite

# Starter idraulico
STARTER_TORQUE    = 150.0   # [N·m] ampiezza coppia di avviamento
STARTER_TAU_BAR   = 60.0    # [bar] costante di tempo decadimento (3τ ≈ 180 bar)

# ── Valvole proporzionali (tau costante di tempo) ────────────────────────────
VALVE_TAU_MOV   = 0.04  # [s] valvola principale LOX (MOV)
VALVE_TAU_MFV   = 0.04  # [s] valvola principale CH4 (MFV)
VALVE_TAU_ORHC  = 0.03  # [s] cross-bleed ORHC
VALVE_TAU_FRHC  = 0.03  # [s] cross-bleed FRHC
VALVE_TAU_AUTO  = 0.05  # [s] valvole autopressurizzazione

# ── Resistenze idrauliche ─────────────────────────────────────────────────────
R_VALVE_OX_K    = 0.00008  # [bar/(kg/s)²] costante resistenza valvola LOX (diviso th³)
R_VALVE_F_K     = 0.00070  # [bar/(kg/s)²] costante resistenza valvola CH4
R_LINE_OX       = 0.0001   # [bar/(kg/s)²] resistenza linea LOX
R_LINE_F        = 0.0005   # [bar/(kg/s)²] resistenza linea CH4
R_JACKET_BASE   = 0.0004   # [bar/(kg/s)²] resistenza jacket (ΔP ≈ 20 bar a regime)

# Cross-bleed
R_BLEED_OX_FRHC = 0.20    # [bar/(kg/s)²] LOX → FRHC
R_BLEED_F_ORHC  = 0.04    # [bar/(kg/s)²] CH4 → ORHC

# ── Iniettori (orifizi fisici, modello Bernoulli) ────────────────────────────
CD_INJ_LIQUID   = 0.92  # Cd iniettori liquidi (pre-burner)
CD_INJ_GAS      = 0.72  # Cd iniettori gas (MCC, scarichi pre-burner)

# Aree iniettori liquidi (pre-burner, flusso principale)
A_INJ_OX_ORHC   = 0.009    # [m²] LOX → ORHC
A_INJ_F_FRHC    = 0.00062  # [m²] CH4 → FRHC (area ridotta per limitare ΔP)

# Aree iniettori liquidi (cross-bleed)
A_INJ_OX_FRHC   = 0.0005   # [m²] LOX bleed → FRHC
A_INJ_F_ORHC    = 0.0007   # [m²] CH4 bleed → ORHC

# Aree iniettori gas (MCC, scarichi pre-burner → camera principale)
A_INJ_GAS_OX    = 0.009    # [m²] gas ossidante → MCC
A_INJ_GAS_F     = 0.010    # [m²] gas combustibile → MCC

# ── Camere di combustione ────────────────────────────────────────────────────
# ORHC (pre-burner ossidante, ox-rich)
ORHC_VOLUME     = 0.015   # [m³]
ORHC_A_THROAT   = 0.007   # [m²]
ORHC_ETA_CSTAR  = 0.98

# FRHC (pre-burner combustibile, fuel-rich)
FRHC_VOLUME     = 0.015
FRHC_A_THROAT   = 0.004
FRHC_ETA_CSTAR  = 0.98

# MCC (camera principale)
MCC_VOLUME      = 0.05
MCC_A_THROAT    = 0.050
MCC_EPS_NOZZLE  = 40.0
MCC_ETA_CSTAR   = 0.97

# Back-pressure stimata dai pre-burner verso MCC
P_BACK_FACTOR   = 0.65    # p_back ≈ P_preburner × P_BACK_FACTOR

# ── Jacket di raffreddamento ─────────────────────────────────────────────────
JACKET_MASS     = 65.0     # [kg]
JACKET_T_INLET  = 110.0    # [K]
JACKET_H_A_BASE = 1200.0   # [W/m²K] HTC lato gas a pressione nominale
JACKET_T_FLAME  = 3600.0   # [K]
JACKET_H_COOL   = 85000.0  # [W/m²K] HTC lato refrigerante nominale
JACKET_MDOT_NOM = 160.0    # [kg/s]  portata nominale refrigerante

# ── Autopressurizzazione serbatoi ────────────────────────────────────────────
AUTOG_RATE      = 5.0    # [bar/s] aumento pressione per unità apertura valvola
AUTOG_BLEED_OX  = 0.0005 # [bar·s/kg] coefficiente perdita LOX
AUTOG_BLEED_F   = 0.0015 # [bar·s/kg] coefficiente perdita CH4
TARGET_P_TANK   = 6.5    # [bar] setpoint pressione serbatoio (avionics)

# ── Avionics: soglie di abort ────────────────────────────────────────────────
ABORT_P_MCC_BAR    = 380.0    # [bar] sovrapressione MCC
ABORT_RPM_LIMIT    = 280000   # [RPM] overspeed turbina
ABORT_T_COOL_K     = 8500.0   # [K]   sovratemperatura refrigerante
ABORT_P_TANK_BAR   = 8.0      # [bar] sovrapressione serbatoio

# ── Avionics: PID gains ───────────────────────────────────────────────────────
PID_THRUST_KP  = 0.00015
PID_THRUST_KI  = 0.0008

PID_MIXTURE_KP = 0.05
PID_MIXTURE_KI = 0.06

PID_ORHC_KP    = 0.001
PID_ORHC_KI    = 1e-3

PID_FRHC_KP    = 0.001
PID_FRHC_KI    = 1e-3

PID_TANK_KP    = 2.0
PID_TANK_KI    = 0.5

PID_CHILL_KP   = 0.005
PID_CHILL_KI   = 0.001

# ── Avionics: target operativi ───────────────────────────────────────────────
TARGET_OF_MCC       = 3.4   # rapporto O/F camera principale
TARGET_OF_ORHC      = 60.0  # rapporto O/F pre-burner ossidante (range 50-80)
TARGET_OF_FRHC      = 0.40  # rapporto O/F pre-burner combustibile (range 0.3-0.5)
TARGET_T_CHILLDOWN  = 120.0 # [K] temperatura target chilldown refrigerante

# Trim valvole cross-bleed in MAIN_STAGE
TRIM_OF_MCC_MAX     = 0.15  # max trim valvole MOV/MFV per controllo OF_MCC
TRIM_ORHC_MAX       = 0.15  # max trim cross-bleed ORHC
TRIM_FRHC_MAX       = 0.15  # max trim cross-bleed FRHC
VALVE_ORHC_NOMINAL  = 0.20  # posizione nominale v_orhc in MAIN_STAGE
VALVE_FRHC_NOMINAL  = 0.60  # posizione nominale v_frhc in MAIN_STAGE

# ── Avionics: sequenza accensione ────────────────────────────────────────────
SEQ_CHILLDOWN_T  = 2.0    # [s] durata chilldown
SEQ_SPINPRIME_T  = 1.5    # [s] durata spin/prime
SEQ_BOOTSTRAP_TO = 8.0    # [s] timeout IGNITION → ABORT "BOOTSTRAP FAILED"
SEQ_IGNITION_RPM_F  = 3000.0  # [RPM] soglia fuel pump per transizione IGNITION→RAMP_UP
SEQ_IGNITION_RPM_OX = 1500.0  # [RPM] soglia ox pump per transizione IGNITION→RAMP_UP
SEQ_RAMPUP_THRUST   = 2600.0  # [kN] early-exit RAMP_UP se spinta raggiunta
SEQ_RAMPUP_DURATION = 5.0     # [s]  durata rampa RAMP_UP
THRUST_RATE_KN_S    = 1200.0  # [kN/s] max rate di variazione spinta in MAIN_STAGE

# Aperture valvole durante IGNITION
IGN_TH_MOV  = 0.40
IGN_TH_MFV  = 0.65
IGN_V_FRHC  = 0.60
IGN_V_ORHC  = 0.08

# Aperture valvole durante RAMP_UP (inizio → fine)
RAMP_TH_MOV_0 = 0.40; RAMP_TH_MOV_1 = 0.68
RAMP_TH_MFV_0 = 0.65; RAMP_TH_MFV_1 = 0.77
RAMP_V_FRHC_0 = 0.60; RAMP_V_FRHC_1 = 0.60  # stabile
RAMP_V_ORHC_0 = 0.08; RAMP_V_ORHC_1 = 0.20
