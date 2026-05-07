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
# Testa pompa: P_dh = p_inlet + K_HEAD_* * RPM²  [bar]
# Sizing: ΔP = P_pump_out - P_tank = 413.5 bar
#   LOX  RPM_des = 22000  → K_HEAD_OX = 413.5 / 22000² = 8.54e-7
#   CH4  RPM_des = 30000  → K_HEAD_F  = 413.5 / 30000² = 4.59e-7
K_HEAD_OX = 1.4742e-6  # [bar/RPM²] LOX pump  (RPM_des=22000, P_pump=720bar)
K_HEAD_F  = 7.9278e-7  # [bar/RPM²] CH4 pump  (RPM_des=30000, P_pump=720bar)

# Turbopompa LOX
TP_OX_INERTIA   = 6.750    # [kg·m²·30/π] inerzia LOX
TP_OX_PUMP_K1   = 4.985e-5 # coppia quadratica [N·m/RPM²]     — sizing 22000 RPM, 47.5 MW
TP_OX_PUMP_K2   = 3.909e-4 # coppia lineare    [N·m/(RPM·bar)] — K2*RPM*P_bar
TP_OX_NPSH      = 5.897e-8 # coefficiente NPSH [bar/RPM²]      — σ=0.04, H=255m

# Turbopompa LCH4
TP_F_INERTIA    = 5.726    # [kg·m²·30/π] inerzia CH4
TP_F_PUMP_K1    = 9.961e-6 # coppia quadratica [N·m/RPM²]     — sizing 30000 RPM, 37.8 MW
TP_F_PUMP_K2    = 1.672e-4 # coppia lineare    [N·m/(RPM·bar)]
TP_F_NPSH       = 3.171e-8 # coefficiente NPSH [bar/RPM²]      — σ=0.04, H=690m

# Efficienza isentropica turbina
TP_ETA_TURBINE  = 0.70

# Freno overspeed (RPM limite design)
TP_OVERSPEED_RPM  = 38000.0
TP_OVERSPEED_K    = 100.0   # [N·m/RPM²] penalità quadratica oltre il limite

# Starter idraulico
STARTER_TORQUE    = 10000.0 # [N·m] coppia di avviamento — sizing: 0.40×10kN·m=4kN·m > τ_pump_ox@7000RPM
STARTER_TAU_BAR   = 200.0   # [bar] costante decadimento — efficace fino a ~200 bar preburner

# ── Valvole proporzionali (tau costante di tempo) ────────────────────────────
VALVE_TAU_MOV   = 0.04  # [s] valvola principale LOX (MOV)
VALVE_TAU_MFV   = 0.04  # [s] valvola principale CH4 (MFV)
VALVE_TAU_ORHC  = 0.03  # [s] cross-bleed ORHC
VALVE_TAU_FRHC  = 0.03  # [s] cross-bleed FRHC
VALVE_TAU_AUTO  = 0.05  # [s] valvole autopressurizzazione

# ── Resistenze idrauliche ─────────────────────────────────────────────────────
R_VALVE_OX_K    = 0.000016  # [bar/(kg/s)²] sizing: ΔP=10bar @ th=0.8, mdot_ox=570 kg/s
R_VALVE_F_K     = 0.000182  # [bar/(kg/s)²] sizing: ΔP=10bar @ th=0.8, mdot_f=168 kg/s
R_LINE_OX       = 0.000015  # [bar/(kg/s)²] ΔP=5bar @ mdot_ox=570 kg/s
R_LINE_F        = 0.000178  # [bar/(kg/s)²] ΔP=5bar @ mdot_f=168 kg/s
R_JACKET_BASE   = 0.000788  # [bar/(kg/s)²] ΔP=20bar @ mdot_cool=159 kg/s

# Cross-bleed
R_BLEED_OX_FRHC = 0.01826  # [bar/(kg/s)²] LOX → FRHC  (31.4 kg/s @ ΔP=50bar, v_frhc_nom=0.60)
R_BLEED_F_ORHC  = 0.01715  # [bar/(kg/s)²] CH4 → ORHC  (10.8 kg/s @ ΔP=50bar, v_orhc_nom=0.20)

# ── Iniettori (orifizi fisici, modello Bernoulli) ────────────────────────────
CD_INJ_LIQUID   = 0.92  # Cd iniettori liquidi (pre-burner)
CD_INJ_GAS      = 0.72  # Cd iniettori gas (MCC, scarichi pre-burner)

# Aree iniettori liquidi (pre-burner, flusso principale)
A_INJ_OX_ORHC   = 0.00627  # [m²] LOX → ORHC  (retargeted per OF=3.4 @ ΔP reale)
A_INJ_F_FRHC    = 0.00193  # [m²] CH4 → FRHC  (retargeted per OF=3.4 @ ΔP reale)

# Aree iniettori liquidi (cross-bleed)
A_INJ_OX_FRHC   = 0.000319 # [m²] LOX bleed → FRHC  (31.4 kg/s, ΔP=50bar)
A_INJ_F_ORHC    = 0.000180 # [m²] CH4 bleed → ORHC  (10.8 kg/s, ΔP=50bar)

# Aree iniettori gas (MCC, scarichi pre-burner → camera principale)
A_INJ_GAS_OX    = 0.01705  # [m²] ORHC gas → MCC  (550 kg/s, ΔP=55bar, Cd=0.72)
A_INJ_GAS_F     = 0.00964  # [m²] FRHC gas → MCC  (188 kg/s, ΔP=55bar, Cd=0.72)

# ── Camere di combustione ────────────────────────────────────────────────────
# ORHC (pre-burner ossidante, ox-rich)
# c*_reale @ OF=60, T≈700K (quasi puro O2 caldo) ≈ 630 m/s — NON 1050 m/s (stoichiometrico)
# A = ṁ·c* / Pc = 550·630 / (670e5) = 0.00517 m²
ORHC_VOLUME     = 0.00414  # [m³]  L*=0.8m @ A_throat=0.00517m²
ORHC_A_THROAT   = 0.00517  # [m²]  sizing: 550 kg/s, c*=630 m/s, Pc=670bar
ORHC_ETA_CSTAR  = 0.98

# FRHC (pre-burner combustibile, fuel-rich)
# c*_reale @ OF=0.4, T≈1400K (fuel-rich CH4 dominante) ≈ 1412 m/s — NON 900 m/s
# A = ṁ·c* / Pc = 188·1412 / (670e5) = 0.00396 m²
FRHC_VOLUME     = 0.00317  # [m³]  L*=0.8m @ A_throat=0.00396m²
FRHC_A_THROAT   = 0.00396  # [m²]  sizing: 188 kg/s, c*=1412 m/s, Pc=670bar
FRHC_ETA_CSTAR  = 0.98

# MCC (camera principale)
MCC_VOLUME      = 0.06856  # [m³]  L*=1.5m @ A_throat=0.04570m²
MCC_A_THROAT    = 0.04570  # [m²]  sizing: 738 kg/s, c*=1858 m/s, Pc=300bar
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
JACKET_MDOT_NOM = 159.3    # [kg/s]  portata nominale refrigerante (95% CH4)

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

PID_TANK_KP    = 1.0
PID_TANK_KI    = 0.1

PID_CHILL_KP   = 0.005
PID_CHILL_KI   = 0.001

# ── Avionics: target operativi ───────────────────────────────────────────────
TARGET_OF_MCC       = 3.4   # rapporto O/F camera principale
TARGET_OF_ORHC      = 31.0  # rapporto O/F pre-burner ossidante (punto operativo naturale)
TARGET_OF_FRHC      = 0.40  # rapporto O/F pre-burner combustibile (range 0.3-0.5)
TARGET_T_CHILLDOWN  = 120.0 # [K] temperatura target chilldown refrigerante

# Trim valvole cross-bleed in MAIN_STAGE
TRIM_OF_MCC_MAX     = 0.25  # max trim valvole MOV/MFV per controllo OF_MCC
TRIM_ORHC_MAX       = 0.15  # max trim cross-bleed ORHC
TRIM_FRHC_MAX       = 0.15  # max trim cross-bleed FRHC
VALVE_ORHC_NOMINAL  = 0.20  # posizione nominale v_orhc in MAIN_STAGE
VALVE_FRHC_NOMINAL  = 0.60  # posizione nominale v_frhc in MAIN_STAGE

# ── Avionics: sequenza accensione ────────────────────────────────────────────

# CHILLDOWN → SPIN_PRIME
# Condizione primaria: T_cool ≤ soglia (propellante freddo a sufficienza)
# Guard timer: tempo minimo anche se il sensore è già soddisfatto (evita falsi positivi)
SEQ_CHILLDOWN_T_COOL_MAX = 125.0  # [K]   T_cool deve scendere sotto questa soglia
SEQ_CHILLDOWN_T_MIN      = 1.0    # [s]   tempo minimo nella fase (guard)
SEQ_CHILLDOWN_TO         = 30.0   # [s]   timeout → abort "CHILLDOWN TIMEOUT"

# SPIN_PRIME → IGNITION
# Condizione primaria: RPM pompe > soglia (linee idrauliche in pressione)
# Condizione secondaria: pressioni serbatoi OK
SEQ_SPINPRIME_RPM_OX_MIN = 7000.0  # [RPM] pompa LOX: pressione sufficiente all'ingresso ORHC
SEQ_SPINPRIME_RPM_F_MIN  = 12000.0 # [RPM] pompa CH4: pressione sufficiente all'ingresso FRHC
SEQ_SPINPRIME_RPM_MIN    = 200.0   # [RPM] legacy (non usato in avionics, tenuto per compatibilità)
SEQ_SPINPRIME_P_TANK_MIN = 5.0     # [bar] entrambi i serbatoi pressurizati
SEQ_SPINPRIME_T_MIN      = 0.5     # [s]   tempo minimo nella fase (guard)
SEQ_SPINPRIME_TO         = 20.0    # [s]   timeout → abort "SPIN_PRIME TIMEOUT" (allungato per spinup)

# IGNITION → RAMP_UP (bootstrap termico confermato)
SEQ_IGNITION_RPM_F  = 3000.0  # [RPM] fuel pump
SEQ_IGNITION_RPM_OX = 1500.0  # [RPM] ox pump
SEQ_BOOTSTRAP_TO    = 8.0     # [s]   timeout → abort "BOOTSTRAP FAILED"

# RAMP_UP → MAIN_STAGE
SEQ_RAMPUP_THRUST   = 2600.0  # [kN] early-exit se spinta raggiunta
SEQ_RAMPUP_DURATION = 5.0     # [s]  durata massima rampa
THRUST_RATE_KN_S    = 1200.0  # [kN/s] max rate variazione spinta in MAIN_STAGE

# Aperture valvole durante IGNITION
# th_mov ridotto: evita P_ORHC > P_pump_f durante bootstrap
# v_frhc ridotto: evita che LOX bleed sommerga FRHC prima che CH4 pump sia in pressione
# v_orhc alzato: più CH4 bleed verso ORHC per avviare combustione ox-rich
IGN_TH_MOV  = 0.20
IGN_TH_MFV  = 0.65
IGN_V_FRHC  = 0.08
IGN_V_ORHC  = 0.30

# Aperture valvole durante RAMP_UP (inizio → fine)
# MOV più alto per chiudere il gap di spinta, MFV più chiusa per OF≈3.4
RAMP_TH_MOV_0 = 0.45; RAMP_TH_MOV_1 = 0.88
RAMP_TH_MFV_0 = 0.45; RAMP_TH_MFV_1 = 0.52
RAMP_V_FRHC_0 = 0.60; RAMP_V_FRHC_1 = 0.60  # stabile
RAMP_V_ORHC_0 = 0.08; RAMP_V_ORHC_1 = 0.20
