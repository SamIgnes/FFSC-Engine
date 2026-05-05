"""
Generatore di tabella termochimica 3D CH4/LOX via CEA-Wrap.

Uso:
    .venv/bin/python3 generate_cea_table.py

Output:
    - Salva 'cea_table_ch4_lox_3d.npz':
        assi: of_points [N_OF], t_fuel_points [N_TF]
        dati: t_ad[N_OF, N_TF], c_star[N_OF, N_TF], gamma[N_OF, N_TF], mw[N_OF, N_TF]
    - Mantiene compatibilità con il vecchio 'cea_table_ch4_lox.npz' (slice a T_ref=112K)

Architettura:
    La terza dimensione T_fuel_in modella il riscaldamento del CH4 nel cooling
    jacket prima di entrare nel FRHC pre-burner (percorso reale FFSCC:
    Tank → Pompa → Jacket → FRHC → Turbina → Iniettori MCC).
"""

import numpy as np
from CEA_Wrap import Fuel, Oxidizer, RocketProblem

# ── Configurazione ────────────────────────────────────────────────────────────
T_OX_K    =  92.0     # temperatura LOX [K] — non varia (non passa dal jacket)
P_REF_BAR = 200.0     # pressione di riferimento [bar]
EPS_MCC   =  40.0     # rapporto di espansione ugello MCC [-]

# Griglia OF
OF_POINTS = np.array([
    0.15, 0.20, 0.30, 0.40, 0.60, 0.80, 1.00,
    1.50, 2.00, 2.50, 3.00, 3.40, 3.60, 4.00,
    5.00, 6.00, 8.00, 10.0, 15.0, 20.0, 30.0,
    40.0, 50.0, 70.0, 100.0, 150.0, 200.0,
])

# Griglia temperatura ingresso CH4 [K]: da criogenico (tank) a ben riscaldato (post-jacket)
# CH4(L) esiste in CEA solo tra ~101.64 e 121.64 K → sopra si usa CH4 (gas/supercrit.)
# 111K → CH4(L), 200K+ → CH4(gas). Gap 122-199K non supportato da CEA → saltato.
T_FUEL_POINTS = np.array([111., 200., 250., 300., 350., 400., 450., 500.])
CH4_LIQ_MAX   = 121.64  # K — limite superiore CH4(L) in CEA

P_PSIA = P_REF_BAR * 14.5038
ox = Oxidizer('O2(L)', temp=T_OX_K)

N_OF = len(OF_POINTS)
N_TF = len(T_FUEL_POINTS)

# Tabelle 2D: asse 0 = OF, asse 1 = T_fuel
t_ad_3d  = np.zeros((N_OF, N_TF))
cstar_3d = np.zeros((N_OF, N_TF))
gamma_3d = np.zeros((N_OF, N_TF))
mw_3d    = np.zeros((N_OF, N_TF))

print(f"CEA-Wrap 3D CH4/LOX — P_ref={P_REF_BAR} bar, T_ox={T_OX_K}K")
print(f"OF grid: {N_OF} punti   T_fuel grid: {N_TF} punti   Totale: {N_OF*N_TF} run")
print("-" * 72)

for j, t_f in enumerate(T_FUEL_POINTS):
    # Usa CH4(L) solo nel range liquido, altrimenti CH4 gassoso/supercritico
    if t_f <= CH4_LIQ_MAX:
        fuel = Fuel('CH4(L)', temp=float(t_f))
    else:
        fuel = Fuel('CH4', temp=float(t_f))
    print(f"\n  T_fuel = {t_f:.0f} K")
    print(f"  {'O/F':>7} {'T_ad(K)':>10} {'c*(m/s)':>10} {'gamma':>8} {'MW':>8}")
    for i, of in enumerate(OF_POINTS):
        try:
            prob = RocketProblem(
                pressure=P_PSIA,
                materials=[fuel, ox],
                o_f=float(of),
                sup=EPS_MCC,
                analysis_type='equilibrium',
            )
            res = prob.run()
            t_ad_3d[i, j]  = res.c_t
            cstar_3d[i, j] = res.cstar
            gamma_3d[i, j] = res.c_gammas
            mw_3d[i, j]    = res.c_mw
            print(f"  {of:7.2f} {res.c_t:10.1f} {res.cstar:10.1f} {res.c_gammas:8.4f} {res.c_mw:8.3f}")
        except Exception as e:
            # Fallback: interpolazione dalla colonna precedente o da valori noti
            if j > 0:
                t_ad_3d[i, j]  = t_ad_3d[i, j-1]
                cstar_3d[i, j] = cstar_3d[i, j-1]
                gamma_3d[i, j] = gamma_3d[i, j-1]
                mw_3d[i, j]    = mw_3d[i, j-1]
            print(f"  {of:7.2f}  [WARN] {e} — copiato da T_fuel precedente")

print("\n" + "-" * 72)

# ── Salva tabella 3D ──────────────────────────────────────────────────────────
np.savez(
    'cea_table_ch4_lox_3d.npz',
    of_points=OF_POINTS,
    t_fuel_points=T_FUEL_POINTS,
    t_ad=t_ad_3d,
    c_star=cstar_3d,
    gamma=gamma_3d,
    mw=mw_3d,
    p_ref_bar=np.array([P_REF_BAR]),
    eps_mcc=np.array([EPS_MCC]),
)
print("Salvato: cea_table_ch4_lox_3d.npz")

# ── Mantieni compatibilità: salva anche la slice 2D a T_ref=112K ─────────────
j_ref = int(np.argmin(np.abs(T_FUEL_POINTS - 112.0)))
data_2d = np.column_stack([
    OF_POINTS,
    t_ad_3d[:, j_ref],
    cstar_3d[:, j_ref],
    gamma_3d[:, j_ref],
    mw_3d[:, j_ref],
])
np.savez(
    'cea_table_ch4_lox.npz',
    of=data_2d[:, 0], t_ad=data_2d[:, 1], c_star=data_2d[:, 2],
    gamma=data_2d[:, 3], mw=data_2d[:, 4],
    p_ref_bar=np.array([P_REF_BAR]),
    eps_mcc=np.array([EPS_MCC]),
)
print("Salvato: cea_table_ch4_lox.npz  (slice 2D a T_fuel≈112K, backward-compat)")
