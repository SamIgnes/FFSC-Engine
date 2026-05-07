"""
sim_50s.py
Simulazione headless 50 s – usa i moduli separati.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from core.engine import FFSCC_Engine
from core.avionics import FlightComputer
from config import SIM_DT as DT, SIM_T_TOTAL as T_TOTAL, SIM_START_SEQ as START_SEQ, SIM_TARGET_KN as TARGET_KN, K_HEAD_OX, K_HEAD_F

N      = int(T_TOTAL / DT)
engine = FFSCC_Engine()
fc     = FlightComputer(dt=DT)

STATE_MAP = {
    "IDLE": 0, "CHILLDOWN": 1, "SPIN_PRIME": 2, "IGNITION": 3,
    "RAMP_UP": 4, "MAIN_STAGE": 5, "MECO": 6, "ABORT": 7,
}
state_colors = {
    0: '#bdc3c7', 1: '#3498db', 2: '#f39c12', 3: '#e67e22',
    4: '#e8b500', 5: '#27ae60', 6: '#8e44ad', 7: '#c0392b',
}

# ── Arrays di output ──────────────────────────────────────────────────
keys = ['t', 'thrust', 'p_mcc', 't_cool', 'w_ox', 'w_f',
        'p_orhc', 'p_frhc', 'p_tank_ox', 'p_tank_f',
        'v_mov', 'v_mfv', 'v_auto_f', 'state_id']
data = {k: np.zeros(N) for k in keys}

# ── Loop di simulazione ───────────────────────────────────────────────
print(f"{'t':>6}  {'stato':<12}  {'Spinta':>8}  {'P_MCC':>7}  {'T_cool':>7}  {'Ox RPM':>7}  {'Fuel RPM':>8}")
print("-" * 68)

for i in range(N):
    t = i * DT

    if t >= START_SEQ and fc.state == "IDLE":
        fc.start_sequence()

    curr_thrust = engine.get_current_thrust()
    st = engine.state

    telemetry  = (st[0] / 1e5, st[1], st[2], st[7],
                  engine.of_orhc_current, engine.of_frhc_current, st[8], st[9],
                  engine.mdot_ox_last, engine.mdot_f_last,
                  engine.p_dh_ox_bar, engine.p_dh_f_bar,
                  st[12] / 1e5, st[13] / 1e5)
    cav_status = (engine.tp_ox.is_cavitating, engine.tp_fuel.is_cavitating)

    th_mov, th_mfv, v_orhc, v_frhc, v_auto_ox, v_auto_f, is_ignited, state_str = \
        fc.update(TARGET_KN, curr_thrust, telemetry, cav_status)

    engine.cmd_th_mov    = th_mov
    engine.cmd_th_mfv    = th_mfv
    engine.cmd_v_orhc    = v_orhc
    engine.cmd_v_frhc    = v_frhc
    engine.cmd_v_auto_ox = v_auto_ox
    engine.cmd_v_auto_f  = v_auto_f
    engine.is_ignited    = is_ignited

    # Salva
    data['t'][i]          = t
    data['thrust'][i]     = curr_thrust
    data['p_mcc'][i]      = st[0] / 1e5
    data['t_cool'][i]     = st[7]
    data['w_ox'][i]       = st[1]
    data['w_f'][i]        = st[2]
    data['p_orhc'][i]     = st[12] / 1e5
    data['p_frhc'][i]     = st[13] / 1e5
    data['p_tank_ox'][i]  = st[8]
    data['p_tank_f'][i]   = st[9]
    data['v_mov'][i]      = st[3] * 100
    data['v_mfv'][i]      = st[4] * 100
    data['v_auto_f'][i]   = st[11] * 100
    data['state_id'][i]   = STATE_MAP.get(state_str, -1)

    engine.step(DT)

    abort_info = f"  [{fc.abort_reason}]" if state_str == "ABORT" and fc.abort_reason else ""
    # Stampa densa durante bootstrap (IGNITION/RAMP_UP) e ogni 5s altrimenti
    in_bootstrap = state_str in ("IGNITION", "RAMP_UP", "SPIN_PRIME")
    state_changed = (i > 0 and data['state_id'][i-1] != STATE_MAP.get(state_str, -1))
    if in_bootstrap or state_changed or i % 100 == 0 or state_str == "ABORT":
        p_orhc_bar = st[12] / 1e5
        p_frhc_bar = st[13] / 1e5
        p_pump_ox  = st[8] + K_HEAD_OX * st[1]**2
        p_pump_f   = st[9] + K_HEAD_F  * st[2]**2
        print(f"{t:6.2f}s  {state_str:<12}  {curr_thrust:8.1f}kN  "
              f"P_MCC={st[0]/1e5:6.1f}  Ppump_ox={p_pump_ox:6.1f}  Ppump_f={p_pump_f:6.1f}  "
              f"P_ORHC={p_orhc_bar:6.1f}  P_FRHC={p_frhc_bar:6.1f}  "
              f"Ox={st[1]:7.0f}RPM  Fuel={st[2]:7.0f}RPM  "
              f"OF_orhc={engine.of_orhc_current:5.1f}  OF_frhc={engine.of_frhc_current:5.3f}  "
              f"T_orhc={engine.t_orhc_current:6.0f}K  T_frhc={engine.t_frhc_current:6.0f}K{abort_info}")

print("-" * 68)
print(f"Spinta finale: {data['thrust'][-1]:.1f} kN  |  "
      f"T_cool finale: {data['t_cool'][-1]:.1f} K  |  "
      f"P_MCC finale: {data['p_mcc'][-1]:.1f} bar")

# ── Plot 4 pannelli ───────────────────────────────────────────────────
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
fig.suptitle("FFSCC – Simulazione headless 50 s", fontweight='bold', fontsize=13)

# Sfondo colorato per stato
for ax in (ax1, ax2, ax3, ax4):
    prev_sid  = data['state_id'][0]
    seg_start = data['t'][0]
    for j in range(1, N):
        if data['state_id'][j] != prev_sid or j == N - 1:
            col = state_colors.get(int(prev_sid), '#ffffff')
            ax.axvspan(seg_start, data['t'][j], alpha=0.08, color=col, linewidth=0)
            seg_start = data['t'][j]
            prev_sid  = data['state_id'][j]

# Pannello 1: Spinta + Pressure Ladder
ax1b = ax1.twinx()
ax1.plot(data['t'], data['thrust'], 'b-',  lw=2,   label='Spinta (kN)')
ax1.axhline(TARGET_KN, color='k', ls='--', lw=1,   label='Target')
ax1b.plot(data['t'], data['p_mcc'],  color='purple', lw=1.8, label='P_MCC')
ax1b.plot(data['t'], data['p_orhc'], 'c--', lw=1.2, alpha=0.8, label='P_ORHC')
ax1b.plot(data['t'], data['p_frhc'], 'g--', lw=1.2, alpha=0.8, label='P_FRHC')
ax1.set_ylabel('Spinta (kN)')
ax1b.set_ylabel('Pressione (bar)')
ax1.legend(loc='upper left', fontsize=8)
ax1b.legend(loc='upper right', fontsize=8)
ax1.set_title('Spinta & Cascata di Pressione', fontweight='bold')
ax1.grid(True, alpha=0.4)

# Pannello 2: RPM + T_cool
ax2b = ax2.twinx()
ax2.plot(data['t'], data['w_ox'], 'c-', lw=2, label='Ox RPM')
ax2.plot(data['t'], data['w_f'],  'g-', lw=2, label='Fuel RPM')
ax2b.plot(data['t'], data['t_cool'], color='#e17055', lw=2, label='T_cool (K)')
ax2b.axhline(120.0, color='orange', ls=':', lw=1.2, label='Target chilldown (120K)')
ax2.set_ylabel('RPM')
ax2b.set_ylabel('T_cool (K)', color='#e17055')
ax2b.tick_params(axis='y', labelcolor='#e17055')
ax2.legend(loc='upper left', fontsize=8)
ax2b.legend(loc='upper right', fontsize=8)
ax2.set_title('Turbopompe & Temperatura Refrigerante', fontweight='bold')
ax2.grid(True, alpha=0.4)

# Pannello 3: Pressioni serbatoi
ax3.plot(data['t'], data['p_tank_ox'], 'c-', lw=2, label='P_tank LOX')
ax3.plot(data['t'], data['p_tank_f'],  'g-', lw=2, label='P_tank LCH4')
ax3.axhline(4.0, color='gray', ls=':', lw=1, label='Target autogeno (4 bar)')
ax3.set_ylabel('Pressione (bar)')
ax3.legend(fontsize=8)
ax3.set_title('Pressioni Serbatoi', fontweight='bold')
ax3.grid(True, alpha=0.4)

# Pannello 4: Valvole
ax4.plot(data['t'], data['v_mov'],    '#0984e3', lw=2,   label='MOV (Ox %)')
ax4.plot(data['t'], data['v_mfv'],    '#d63031', lw=2,   label='MFV (Fuel %)')
ax4.plot(data['t'], data['v_auto_f'], 'g--',     lw=1.5, label='V_auto_f (%)')
ax4.set_ylabel('Apertura (%)')
ax4.set_xlabel('Tempo (s)')
ax4.legend(fontsize=8)
ax4.set_title('Comandi Valvole', fontweight='bold')
ax4.grid(True, alpha=0.4)

plt.tight_layout()
OUT = '/Users/samueleignesti/Desktop/Motor/sim_50s.png'
plt.savefig(OUT, dpi=130, bbox_inches='tight')
print(f"\nPlot salvato in: {OUT}")