# Motor v2.1 — Digital Twin FFSCC (CH4/LOX)

## Cos'è il progetto

Simulatore real-time di un motore a razzo **Full-Flow Staged Combustion Cycle (FFSCC)** alimentato a **metano liquido (LCH4) e ossigeno liquido (LOX)**. Il simulatore implementa un digital twin completo con GUI interattiva multi-thread, controllo avionico PI, modelli fisici ad alta fedeltà, e analisi strutturale con Margini di Sicurezza (MoS).

Target di spinta nominale: **2750 kN**.

---

## Struttura del progetto

```
Motor/
├── main.py                    # Entry point — avvia la GUI Tkinter
├── config.py                  # Parametri fisici centralizzati (tutte le costanti del motore)
├── sim_50s.py                 # Simulazione headless 50 s (no GUI, salva grafici PNG)
├── CLAUDE.md                  # Documentazione progetto
├── TECHNICAL_NOTES.md         # Note tecniche
│
├── core/                      # Moduli fisici del motore (pacchetto)
│   ├── __init__.py
│   ├── engine.py              # FFSCC_Engine — ODE completo a 14 stati con pressure ladder
│   ├── avionics.py            # FlightComputer, PI_Controller, ProportionalValve
│   ├── subsystems.py          # Turbopump, CoolingJacket, CombustionChamber
│   ├── thermodynamics.py      # CH4RealGasProps (CoolProp), CEA_MethaloxCombustion, PropellantPhysics
│   ├── nozzle.py              # SpacialNozzleModel — modello termico 1D ugello (4 nodi radiali)
│   ├── structures.py          # StructuralAnalyzer — MoS per parete, turbine, pre-burner, gola
│   └── cea_table_ch4_lox_3d.npz  # Lookup table termochimica 3D (OF × T_fuel)
│
├── gui/
│   ├── __init__.py
│   └── app.py                 # AppGUI — Tkinter + matplotlib live, sim thread dedicato
│
├── tools/
│   └── generate_cea_table.py  # Genera lookup table CH4/LOX via CEA-Wrap (output 3D + 2D compat)
│
├── Sizing/                    # Script di dimensionamento
│   ├── FFSCC_sizing.py
│   ├── NozzleHeatTransfer.py
│   └── TurbopumpSizing.py
│
└── old/                       # Versioni legacy (motor v1.1 → v2.0)
```

---

## Architettura dei moduli

| File | Responsabilità |
|------|---------------|
| `main.py` | Entry point — istanzia `AppGUI` e avvia `tkinter.mainloop()` |
| `config.py` | **Tutte** le costanti fisiche, PID gains, soglie di sequenza, parametri simulazione |
| `core/engine.py` | `FFSCC_Engine` — vettore di stato 14 componenti, `system_equations()`, `step()` con BDF/RK4 fallback, diagnostica pressure ladder |
| `core/avionics.py` | `FlightComputer` (state machine), `PI_Controller` (con trim), `ProportionalValve` (primo ordine) |
| `core/subsystems.py` | `Turbopump` (bilancio coppia termodinamico reale), `CoolingJacket` (0D), `CombustionChamber` (c* model) |
| `core/thermodynamics.py` | `CH4RealGasProps` (CoolProp con fallback gaussiani), `CEA_MethaloxCombustion` (tabella 3D o 2D + correzioni), `PropellantPhysics` |
| `core/nozzle.py` | `SpacialNozzleModel` — modello termico 1D ugello in rame, 4 nodi radiali, raffreddamento bidirezionale dalla gola |
| `core/structures.py` | `StructuralAnalyzer` — calcolo MoS per parete CuCrZr+IN718, dischi turbina Ti-6Al-4V, involucri pre-burner, fatica termica gola |
| `gui/app.py` | `AppGUI` — Tkinter + matplotlib, SimThread dedicato + queue.Queue per comunicazione thread-safe |
| `sim_50s.py` | Simulazione headless 50 s, stampa console + 4 pannelli PNG |
| `tools/generate_cea_table.py` | Genera `cea_table_ch4_lox_3d.npz` via CEA-Wrap (griglia OF × T_fuel) |

---

## Vettore di stato del motore (14 variabili)

```
[0]  P_mcc      [Pa]   pressione camera di combustione principale
[1]  w_ox       [RPM]  velocità turbopompa ossidante
[2]  w_f        [RPM]  velocità turbopompa combustibile
[3]  th_mov     [-]    apertura valvola principale ossidante (MOV)
[4]  th_mfv     [-]    apertura valvola principale combustibile (MFV)
[5]  v_orhc     [-]    bypass cross-bleed verso ORHC
[6]  v_frhc     [-]    bypass cross-bleed verso FRHC
[7]  T_cool     [K]    temperatura refrigerante camera (CH4)
[8]  P_tank_ox  [bar]  pressione serbatoio LOX
[9]  P_tank_f   [bar]  pressione serbatoio LCH4
[10] v_auto_ox  [-]    valvola autopressurizzazione LOX
[11] v_auto_f   [-]    valvola autopressurizzazione CH4
[12] P_orhc     [Pa]   pressione pre-burner ossidante
[13] P_frhc     [Pa]   pressione pre-burner combustibile
```

### Variabili diagnostiche (non di stato, calcolate post-step)

| Campo | Descrizione |
|-------|------------|
| `mdot_ox_last`, `mdot_f_last` | Portate totali OX/F [kg/s] |
| `of_orhc_current`, `of_frhc_current` | Rapporti O/F pre-burner |
| `t_orhc_current`, `t_frhc_current` | Temperature pre-burner [K] |
| `p_dh_ox_bar`, `p_dh_f_bar` | Pressione uscita pompe [bar] |
| `p_man_ox_bar`, `p_man_f_bar` | Pressione manifold pre-burner [bar] |
| `dp_inj_*_bar` | ΔP iniettori (4 gruppi: OX→ORHC, CH4→FRHC, bleed LOX→FRHC, bleed CH4→ORHC, gas→MCC) |
| `T_wall_max` | Temperatura massima parete rame lato gas [K] |

---

## Sequenza di accensione (stati FlightComputer)

`IDLE` → `CHILLDOWN` → `SPIN_PRIME` → `IGNITION` → `RAMP_UP` → `MAIN_STAGE` → `MECO` / `ABORT`

### Condizioni di transizione

| Da → A | Condizione | Timeout |
|--------|-----------|---------|
| CHILLDOWN → SPIN_PRIME | `T_cool ≤ 125 K` + guard 1 s | 30 s |
| SPIN_PRIME → IGNITION | `RPM_ox ≥ 7000` e `RPM_f ≥ 12000` + serbatoi ≥ 5 bar + guard 0.5 s | 20 s |
| IGNITION → RAMP_UP | `RPM_f > 3000` e `RPM_ox > 1500` | 8 s |
| RAMP_UP → MAIN_STAGE | Spinta > 2600 kN o durata 5 s (min 1 s) | — |

### Valvole durante IGNITION

| Valvola | Apertura |
|---------|----------|
| MOV | 0.20 |
| MFV | 0.65 |
| v_frhc | 0.08 |
| v_orhc | 0.30 |

---

## Limiti di abort automatici

| Condizione | Soglia |
|-----------|--------|
| Cavitazione turbopompa | rilevata da NPSH (attualmente disabilitata, commentata) |
| Sovrapressione MCC | > 380 bar |
| Overspeed turbina | > 280 000 RPM |
| Temperatura refrigerante | > 8 500 K |
| Sovrapressione serbatoio | > 8 bar |

---

## Architettura GUI (threading)

```
┌─────────────────┐       Queue(maxsize=60)      ┌─────────────────┐
│  SimThread      │ ──── snapshot immutable ────→ │  GUI Thread     │
│  (20 Hz)        │                              │  (root.after)   │
│  fc.update()    │                              │  _update_gui()  │
│  engine.step()  │                              │  draw_idle()    │
└─────────────────┘                              └─────────────────┘
```

- **SimThread**: esegue `fc.update()` + `engine.step(dt=0.05)` a 20 Hz, deposita snapshot immutabili in `queue.Queue`. Se la coda è piena, scarta il frame più vecchio.
- **GUI Thread**: drena la coda con `root.after(16ms)`, aggiorna label e plot. Rendering Matplotlib throttled a 1 frame ogni 4 chiamate.
- **Pannello destro**: 3 viste toggle — SCHEMA (pressure ladder), NOZZLE (profilo termico 1D), STRUCT (margini di sicurezza).

---

## Analisi strutturale (StructuralAnalyzer)

Calcola il **Margine di Sicurezza (MoS)** per i componenti critici:

```
MoS = (Carico_Ammissibile / Sforzo_Reale) - 1
```

| Componente | Materiale | Meccanismo |
|-----------|-----------|-----------|
| Parete camera (profilo completo) | CuCrZr liner + IN718 camicia | Pressione differenziale + termico |
| Gola ugello | CuCrZr | Fatica termica (ΔT attraverso parete) |
| Rotore turbina OX | Ti-6Al-4V | Sforzo centrifugo (Timoshenko) |
| Rotore turbina FUEL | Ti-6Al-4V | Sforzo centrifugo (Timoshenko) |
| Involucro ORHC | IN718 | Hoop stress sfera |
| Involucro FRHC | IN718 | Hoop stress sfera |

### Semaforo MoS

| MoS | Stato | Colore |
|-----|-------|--------|
| < 0 | FAILURE | Rosso (#e74c3c) |
| 0 – 0.25 | CAUTION | Giallo (#f39c12) |
| ≥ 0.25 | OK | Verde (#2ecc71) |

Fattori di sicurezza: **FS_yield = 1.25**, **FS_rupture = 1.50** (MIL-HDBK-5).

---

## Come avviare

```bash
# GUI interattiva
python main.py

# Simulazione headless 50 s
python sim_50s.py

# Rigenerare tabella CEA (richiede CEA-Wrap installato)
python tools/generate_cea_table.py
```

---

## Dipendenze principali

- `numpy`, `scipy` — ODE (`solve_ivp` BDF), algebra lineare, interpolazione
- `matplotlib` — grafici live (GUI) e headless (PNG)
- `tkinter` — GUI (stdlib Python)
- `CoolProp` — proprietà gas reale CH4 supercritico (opzionale; fallback analitico attivo se assente)
- `CEA-Wrap` — solo per `tools/generate_cea_table.py`, non richiesto a runtime

---

## Note architetturali

- La termochimica a runtime usa un **lookup table 3D interpolato** (`cea_table_ch4_lox_3d.npz`) su (OF, T_fuel_in) per velocità real-time. Se assente, fallback a tabella 2D hardcoded + correzione entalpica analitica per T_fuel.
- `CoolProp` è opzionale: se non installato, `CH4RealGasProps` usa modelli analitici semplificati (gaussiani) con un avviso a console.
- Il passo temporale di default è **dt = 0.05 s** sia in GUI che in headless (20 Hz).
- Il modello ugello `SpacialNozzleModel` è disaccoppiato dall'ODE principale (operator splitting: aggiornato post-step). Raffreddamento bidirezionale dalla gola (50% verso camera, 50% verso exit).
- Tutti i parametri fisici sono centralizzati in `config.py` — nessun magic number nei moduli.
- La cavitazione è modellata ma **disabilitata di default** (commentata in `Turbopump.get_derivative()`).
- Pressure guard: interlock duro che riduce th_mov/v_frhc se la pressione di un pre-burner si avvicina a quella della pompa, impedendo backflow.
