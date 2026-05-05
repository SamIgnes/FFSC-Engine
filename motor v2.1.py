"""
motor v2.1.py
Entry point principale – avvia la GUI del Digital Twin FFSCC.

La logica è ora suddivisa in moduli separati:
    thermodynamics.py  – proprietà CH4/LOX e termochimica CEA
    avionics.py        – FlightComputer, PI_Controller, ProportionalValve
    subsystems.py      – Turbopump, CoolingJacket, CombustionChamber
    engine.py          – FFSCC_Engine (ODE completo con pressure ladder)
    nozzle.py          – SpacialNozzleModel (termico 1D ugello)
    gui.py             – AppGUI (Tkinter + matplotlib live)
"""
from gui import AppGUI
import tkinter as tk

if __name__ == "__main__":
    root = tk.Tk()
    app  = AppGUI(root)
    root.mainloop()
