# Motor v2.1 — Digital Twin FFSCC (CH4/LOX)

## Cos'è il progetto

Simulatore real-time di un motore a razzo **Full-Flow Staged Combustion Cycle (FFSCC)** alimentato a **metano liquido (LCH4) e ossigeno liquido (LOX)**. Il simulatore implementa un digital twin completo con GUI interattiva, controllo avionico PI, e modelli fisici ad alta fedeltà.

Target di spinta nominale: **2500 kN**.

---

## Architettura dei moduli

| File | Responsabilità |
|------|---------------|
| `motor v2.1.py` | Entry point — avvia la GUI Tkinter |
| `thermodynamics.py` | Proprietà termodinamiche CH4/LOX (via CoolProp) e dati termochimici CEA |
| `avionics.py` | `FlightComputer`, `PI_Controller`, `ProportionalValve` |
| `subsystems.py` | `Turbopump`, `CoolingJacket`, `CombustionChamber` |
| `engine.py` | `FFSCC_Engine` — ODE completo a 14 stati con pressure ladder |
| `nozzle.py` | `SpacialNozzleModel` — modello termico 1D ugello (4 nodi radiali in rame) |
| `gui.py` | `AppGUI` — Tkinter + matplotlib live telemetry |
| `sim_50s.py` | Simulazione headless 50 s (no GUI, salva grafici PNG) |
| `generate_cea_table.py` | Genera lookup table termochimica CH4/LOX via CEA-Wrap |

---

## Vettore di stato del motore (14 variabili)

```
[0]  P_mcc      [Pa]   pressione camera di combustione principale
[1]  w_ox       [RPM]  velocità turbopompa ossidante
[2]  w_f        [RPM]  velocità turbopompa combustibile
[3]  th_mov     [-]    apertura valvola principale ossidante (MOV)
[4]  th_mfv     [-]    apertura valvola principale combustibile (MFV)
[5]  v_orhc     [-]    bypass pre-burner ossidante (ORHC)
[6]  v_frhc     [-]    bypass pre-burner combustibile (FRHC)
[7]  T_cool     [K]    temperatura refrigerante camera
[8]  P_tank_ox  [bar]  pressione serbatoio LOX
[9]  P_tank_f   [bar]  pressione serbatoio LCH4
[10] v_auto_ox  [-]    valvola autopressurizzazione LOX
[11] v_auto_f   [-]    valvola autopressurizzazione CH4
[12] P_orhc     [Pa]   pressione pre-burner ossidante
[13] P_frhc     [Pa]   pressione pre-burner combustibile
```

---

## Sequenza di accensione (stati FlightComputer)

`IDLE` → `CHILLDOWN` → `SPIN_PRIME` → `IGNITION` → `RAMP_UP` → `MAIN_STAGE` → `MECO` / `ABORT`

---

## Limiti di abort automatici

| Condizione | Soglia |
|-----------|--------|
| Cavitazione turbopompa | rilevata da NPSH |
| Sovrapressione MCC | > 450 bar |
| Overspeed turbina | > 280 000 RPM |
| Temperatura refrigerante | > 8 500 K |
| Temperatura pre-burner | > 20 000 K |
| Sovrapressione serbatoio | > 8 bar |

---

## Dipendenze principali

- `numpy`, `scipy` — ODE, algebra lineare
- `matplotlib` — grafici live e headless
- `tkinter` — GUI (stdlib Python)
- `CoolProp` — proprietà gas reale CH4 supercritico (opzionale; fallback analitico attivo se assente)
- `CEA-Wrap` — solo per `generate_cea_table.py`, non richiesto a runtime

---

## Come avviare

```bash
# GUI interattiva
python "motor v2.1.py"

# Simulazione headless 50 s
python sim_50s.py

# Rigenerare tabella CEA (richiede CEA-Wrap installato)
python generate_cea_table.py
```

---

## Note architetturali

- La termochimica a runtime usa un **lookup table interpolato** (precalcolato da `generate_cea_table.py`) per velocità real-time; CEA-Wrap non è necessario durante la simulazione.
- `CoolProp` è opzionale: se non installato, `CH4RealGasProps` usa modelli analitici semplificati (gaussiani) con un avviso a console.
- Il passo temporale di default è **dt = 0.05 s** sia in GUI che in headless.
- Il modello ugello `SpacialNozzleModel` è disaccoppiato dall'ODE principale e può essere aggiornato indipendentemente.
