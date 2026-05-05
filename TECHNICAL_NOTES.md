"""DOCUMENTAZIONE TECNICA INTEGRALE: DIGITAL TWIN MOTORE FFSCC (V2.1)

Questo documento descrive analiticamente ogni modello matematico, equazione e costante utilizzata nel simulatore, seguendo la gerarchia di integrazione del sistema.

1. MODELLO TERMODINAMICO E TERMOCHIMICO (thermodynamics.py)

   A. Proprietà del Metano (CH4RealGasProps):
      - Punto Critico: T_crit = 190.564 K, P_crit = 45.992 bar.
      - Modello Cp (Calore Specifico): 
        - Se CoolProp è attivo: Query 'C' basata su T e P (range 91-624 K, 0.1-700 bar).
        - Fallback Analitico: Cp = 3200 (baseline) + 8500 * exp(-0.5 * ((T-200)/25)^2) + (511000/4*sqrt(2pi)) * exp(-0.5 * ((T-111.7)/4)^2).
      - Pressione di Vapore (CH4): 10^(3.98 - 443 / (T - 0.5)) [bar].
      - Etichettatura di Fase: Determina lo stato (GAS, LIQ, DENSE-LIQ, SUPERCRIT) in base al superamento di T_crit e P_crit.

   B. Termochimica CEA (NASA-SP-273):
      - Interpolazione: Lookup table su 23 punti di O/F (da 0.15 a 50.0).
      - Correzione Logaritmica di Pressione:
        - T_ad_corr = T_ad_ref * (1 + 0.015 * log10(P/200))
        - c*_corr = c*_ref * (1 + 0.008 * log10(P/200))
      - Coefficiente di Spinta (Cf): Risoluzione iterativa della relazione Area-Mach per espansione supersonica:
        f(Me) = (1/Me) * [(2/(gamma+1))*(1 + 0.5*(gamma-1)*Me^2)]^((gamma+1)/(2*(gamma-1))) - epsilon = 0.
        Equazione Cf: cf_vac + (p_exit - p_ambient)/p_chamber * epsilon.

2. SOTTOSISTEMI FISICI E DINAMICI (subsystems.py)

   A. Turbopompa (Modello Centrifugo):
      - Derivata Velocità: dw/dt = (Tau_turbina - Tau_pompa) / Inertia.
      - Coppia Pompa: Tau_p = (k1 * w^2 + k2 * w * P_out) * cav_factor.
        - k1_ox=0.00005, k2_ox=0.0005 | k1_f=0.00001, k2_f=0.0002.
      - Modello di Cavitazione: 
        - NPSH_req = 0.7e-8 * w^2.
        - cav_factor = 1.0 - (NPSH_req - NPSH_avail) * 2.0 (clamp min 0.01).
      - Efficienza Aerodinamica: Aero_eff = max(0, 1 - w/35000).

   B. Camicia di Raffreddamento (CoolingJacket):
      - Bilancio Termico: dT/dt = (Q_comb + Q_amb - Q_rem) / (Mass * Cp).
      - Q_combustione: h_eff * (T_flame - T_coolant).
      - Conduttanza h_eff (Serie): 1/h_eff = 1/h_gas + 1/h_cool.
        - h_gas = h_a_base * (P_mcc/101325)^0.8.
        - h_cool = h_cool_nom * (mdot/mdot_nom)^0.8.

   C. Camera di Combustione (CombustionChamber):
      - Dinamica Pressione: dP/dt = (R * T_flame / (MW * Volume)) * (mdot_in - mdot_out).
      - Costante R_gas: 8314 / MW.
      - Scarico Ugello: mdot_out = (P_mcc * A_throat) / c*_eff.

3. AVIONICA E CONTROLLO (avionics.py)

   - Algoritmo PID: output = Kp * error + Ki * integral.
     - Anti-Windup: integral = clamp(-1/Ki, 1/Ki, integral).
   - Logica di Volo:
     - SPIN_PRIME: Target spinta 5% (0.05) per 1.5s.
     - IGNITION: Target spinta 30% (0.30), apertura ORHC 3%.
     - RAMP_UP: Incremento lineare 0.30 -> 0.75 in 1.5s.
     - MAIN_STAGE (Bumpless Transfer): Inizializzazione integrale PID = th_main / Ki per evitare salti al passaggio manuale.
   - Limite di Velocità (Rate Limit): 400 kN/s in MAIN_STAGE.
   - Protezioni (Abort): P_mcc > 420 bar, RPM > 255.000, T_cool > 8500 K (limite software).

4. RETE IDRAULICA E ODE (engine.py)

   Il sistema è descritto da 13 ODE risolte simultaneamente:
   - Pressione Mandata Pompa: P_dh = P_tank + (rho * g * h_tank)/1e5 + K_head * w^2.
     - K_head_ox = 1.2e-6, K_head_f = 0.8e-6.
   - Modello Valvole: R_valve = R_base / (apertura^3 + 1e-6).
     - R_base_ox = 0.00050, R_base_f = 0.0065.
   - Calcolo Portate (mdot): mdot = sqrt( (P_dh - P_preburner_bar) / (R_pump + R_valve) ).
   - Dinamica Pre-Burner:
     - dP_pb/dt = (400 * T_pb / Vol) * (mdot_in - mdot_out).
     - mdot_out_pb = coeff * sqrt(P_pb - P_mcc).

5. MODELLO TERMICO SPAZIALE 1D (nozzle.py)

   - Mach Locale: Risoluzione numerica area-Mach su 100 nodi assiali.
   - Profilo h_gas (Correlazione di Bartz): 
     h_g = (0.026 / D_t^0.2) * (mu^0.2 * Cp / Pr^0.6) * (P_c / c*)^0.8 * (A_t / A)^0.9.
   - Integrazione Radiale (4 Nodi Rame):
     - Nodo Faccia Gas: C_surf * dT/dt = Q_gas - Q_conduzione.
     - Nodi Interni: C_int * dT/dt = Q_cond_in - Q_cond_out.
     - Nodo Faccia Coolant: C_surf * dT/dt = Q_conduzione - Q_convezione_coolant.
   - Raffreddamento Controcorrente: Integrazione spaziale inversa da x_exit a x_inlet:
     T_cool(i-1) = T_cool(i) + Q_net / (mdot * Cp).

6. SEQUENZA LOGICA DEL DIGITAL TWIN
   1. Ricezione target_thrust da GUI.
   2. Flight Computer calcola i comandi valvole (PID) basandosi su telemetria istantanea.
   3. Solver BDF integra le 13 ODE (engine.py) calcolando le portate reali basate sulle curve di pressione.
   4. Il calore generato in MCC viene trasferito al modello nozzle.py.
   5. Il profilo termico aggiorna la temperatura del fluido (T_cool) che rientra nel calcolo della densità del pre-burner (anello di feedback termodinamico).
"""