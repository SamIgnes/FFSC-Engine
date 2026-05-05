import math
try:
    from CoolProp.CoolProp import PropsSI
except ImportError:
    print("Errore: devi installare CoolProp. Esegui 'pip install CoolProp' nel terminale.")
    exit()

class Turbopump:
    def __init__(self, name, fluid, mass_flow, p_in_bar, p_out_bar, t_in_k, rpm, efficiency=0.75, psi=0.5, num_stages=1):
        self.name = name
        self.fluid = fluid           
        self.mass_flow = mass_flow   
        self.p_in = p_in_bar * 1e5   
        self.p_out = p_out_bar * 1e5 
        self.t_in = t_in_k           
        self.rpm = rpm               
        self.efficiency = efficiency
        self.psi = psi               
        self.num_stages = num_stages 
        self.power_w = 0 

    def calculate_sizing(self):
        self.density = PropsSI('D', 'P', self.p_in, 'T', self.t_in, self.fluid)
        self.q = self.mass_flow / self.density
        
        self.delta_p = self.p_out - self.p_in
        self.head_total = self.delta_p / (self.density * 9.81)
        self.head_per_stage = self.head_total / self.num_stages
        
        self.ns_stage = (self.rpm * math.sqrt(self.q)) / (self.head_per_stage ** 0.75)
        
        # Potenza Idraulica Assorbita dalla pompa
        self.power_w = (self.q * self.delta_p) / self.efficiency
        self.power_mw = self.power_w / 1e6
        
        self.u2 = math.sqrt((9.81 * self.head_per_stage) / self.psi)
        omega = self.rpm * math.pi / 30 
        self.d2 = (2 * self.u2) / omega 

    def print_report(self):
        print(f"--- POMPA: {self.name} ({self.fluid}) ---")
        print(f"Stadi:                    {self.num_stages}")
        print(f"Pressione Uscita:         {self.p_out / 1e5:.0f} bar")
        print(f"Potenza RICHIESTA:        {self.power_mw:.2f} MW")
        print(f"Diametro Girante (D2):    {self.d2 * 100:.2f} cm")
        print(f"Velocità Periferica (U2): {self.u2:.2f} m/s")
        print("-" * 55)

class GasTurbine:
    def __init__(self, name, target_power_w, mass_flow, p_in_bar, p_out_bar, t_in_k, rpm, cp_gas, gamma_gas, efficiency=0.82, u_c0_ratio=0.5):
        """
        Inizializza i parametri della turbina a gas usando i valori di input dell'utente.
        """
        self.name = name
        self.target_power_w = target_power_w  # Potenza che la pompa esige
        self.mass_flow = mass_flow
        self.p_in = p_in_bar * 1e5
        self.p_out = p_out_bar * 1e5
        self.t_in = t_in_k                    # Ora è un input fisso
        self.rpm = rpm
        self.cp = cp_gas
        self.gamma = gamma_gas
        self.efficiency = efficiency
        self.u_c0_ratio = u_c0_ratio

    def calculate_sizing(self):
        # 1. Calcolo del Rapporto di Pressione (Expansion Ratio)
        self.expansion_ratio = self.p_in / self.p_out

        # 2. Calcolo Salto Entalpico (Lavoro Specifico) per kg di gas
        # Formula espansione reale: W = Cp * T_in * efficienza * [1 - (P_out/P_in)^((gamma-1)/gamma)]
        pressure_ratio_term = 1 - math.pow((self.p_out / self.p_in), ((self.gamma - 1) / self.gamma))
        self.specific_work = self.cp * self.t_in * self.efficiency * pressure_ratio_term

        # 3. Calcolo Potenza Generata e Bilancio
        self.power_generated_w = self.specific_work * self.mass_flow
        self.power_margin_w = self.power_generated_w - self.target_power_w

        # 4. Temperatura di uscita del gas
        self.t_out = self.t_in - (self.specific_work / self.cp)

        # 5. Dimensionamento Fisico
        delta_h_is = self.specific_work / self.efficiency
        self.c0 = math.sqrt(2 * delta_h_is)
        self.u = self.c0 * self.u_c0_ratio
        omega = self.rpm * math.pi / 30
        self.d_mean = (2 * self.u) / omega

    def print_report(self):
        print(f"--- TURBINA: {self.name} ---")
        print(f"Temperatura Preburner:    {self.t_in:.2f} K")
        print(f"Rapporto Espansione:      {self.expansion_ratio:.2f} ({self.p_in/1e5:.0f} bar -> {self.p_out/1e5:.0f} bar)")
        print(f"Potenza GENERATA:         {self.power_generated_w / 1e6:.2f} MW")
        print(f"Potenza RICHIESTA:        {self.target_power_w / 1e6:.2f} MW (Dalla pompa)")
        
        margin_str = "OK (Esubero)" if self.power_margin_w > 0 else "ATTENZIONE (Deficit)"
        print(f"Margine di Potenza:       {self.power_margin_w / 1e6:+.2f} MW [{margin_str}]")
        
        print(f"Temp. Scarico Gas:        {self.t_out:.2f} K")
        print(f"Velocità Gas (C0):        {self.c0:.2f} m/s")
        print(f"Velocità Pala (U):        {self.u:.2f} m/s")
        print(f"Diametro Medio (D):       {self.d_mean * 100:.2f} cm")
        print("-" * 55)

def main():
    total_mass_flow = 650.0  
    mixture_ratio_of = 3.6   
    m_ch4 = total_mass_flow / (1 + mixture_ratio_of)
    m_lox = total_mass_flow - m_ch4

    # --- 1. POMPE ---
    # La pompa LOX deve superare i 1000 bar per alimentare il pre-bruciatore
    pump_lox = Turbopump("LOX Main Pump", "Oxygen", m_lox, p_in_bar=3.0, p_out_bar=1050.0, t_in_k=90.0, rpm=25000, efficiency=0.74, psi=0.45, num_stages=1)
    
    # La pompa CH4 deve superare i 410 bar
    pump_ch4 = Turbopump("LCH4 Main Pump", "Methane", m_ch4, p_in_bar=3.0, p_out_bar=450.0, t_in_k=110.0, rpm=35000, efficiency=0.76, psi=0.45, num_stages=2)
    
    pump_ch4.calculate_sizing()
    pump_lox.calculate_sizing()

    # --- 2. TURBINE CON PARAMETRI AGGIORNATI ---
    turbine_ch4 = GasTurbine(
        name="Fuel-Rich Turbine (CH4)",
        target_power_w=pump_ch4.power_w, 
        mass_flow=m_ch4,                 
        p_in_bar=410.0,     # Dati utente
        p_out_bar=350.0,    # Pressione Camera Combustione
        t_in_k=972.0,       # Dati utente
        rpm=pump_ch4.rpm,                
        cp_gas=2800.0,                   
        gamma_gas=1.25
    )

    turbine_lox = GasTurbine(
        name="Ox-Rich Turbine (LOX)",
        target_power_w=pump_lox.power_w,
        mass_flow=m_lox,
        p_in_bar=1000.0,    # Dati utente
        p_out_bar=350.0,    # Pressione Camera Combustione
        t_in_k=789.0,       # Dati utente
        rpm=pump_lox.rpm,
        cp_gas=1050.0,
        gamma_gas=1.35
    )

    turbine_ch4.calculate_sizing()
    turbine_lox.calculate_sizing()

    # --- 3. OUTPUT ---
    print("=======================================================")
    print(" SIMULAZIONE 1D BILANCIO POTENZE (POMPE + TURBINE)  ")
    print("=======================================================\n")
    pump_ch4.print_report()
    turbine_ch4.print_report()
    print("\n")
    pump_lox.print_report()
    turbine_lox.print_report()

if __name__ == "__main__":
    main()