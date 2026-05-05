import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

class SpacialNozzleModel:
    def __init__(self):
        # Proprietà termofisiche fisse (approssimate per Methalox)
        self.gamma = 1.14
        self.Pr = 0.7  # Numero di Prandtl del gas
        self.mu = 1.2e-4  # Viscosità dinamica [Pa*s]
        self.cp_gas = 2800.0  # Calore specifico gas [J/kg*K]
        
        # Geometria della parete
        self.t_wall = 0.002  # Spessore parete di rame [m] (2 mm)
        self.k_wall = 350.0  # Conducibilità termica lega di rame [W/m*K]
        
    def generate_geometry(self, x_start=-0.2, x_end=0.6, num_points=200):
        """Genera un profilo semplificato per camera, gola e divergente."""
        x = np.linspace(x_start, x_end, num_points)
        r = np.zeros_like(x)
        
        # Profilo inventato: cilindro, poi cono convergente, gola a x=0, parabola divergente
        R_chamber = 0.15
        R_throat = 0.05
        R_exit = 0.35
        
        for i, xi in enumerate(x):
            if xi < -0.1:
                r[i] = R_chamber
            elif xi <= 0.0:
                # Raccordo lineare fino alla gola
                r[i] = R_throat + (R_chamber - R_throat) * (abs(xi) / 0.1)
            else:
                # Espansione parabolica
                r[i] = R_throat + (R_exit - R_throat) * (xi / x_end)**1.5
                
        return x, r

    def get_mach_from_area(self, A_ratio, gamma, is_supersonic):
        """Risolve la relazione Area-Mach usando fsolve di Scipy."""
        def area_mach_eq(M):
            if M <= 0: return 1e6
            term1 = 1.0 / M
            term2 = (2.0 / (gamma + 1.0)) * (1.0 + 0.5 * (gamma - 1.0) * M**2)
            exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0))
            return term1 * (term2 ** exponent) - A_ratio
            
        M_guess = 3.0 if is_supersonic else 0.1
        M_sol, = fsolve(area_mach_eq, M_guess)
        return M_sol

    def compute_1d_profile(self, P_c_pa, T_c, c_star, h_coolant, T_coolant):
        x, r = self.generate_geometry()
        A = np.pi * r**2
        A_throat = np.min(A)
        D_throat = 2.0 * np.min(r)
        
        # Prealloca array per i risultati
        M = np.zeros_like(x)
        T_gas = np.zeros_like(x)
        P_gas = np.zeros_like(x)
        h_g = np.zeros_like(x)
        q_flux = np.zeros_like(x)
        T_wall_hot = np.zeros_like(x)
        T_wall_cold = np.zeros_like(x)
        
        # 1. Fluidodinamica Isentropica
        for i, xi in enumerate(x):
            A_ratio = A[i] / A_throat
            # Assume che x=0 sia la gola
            is_supersonic = xi > 0.0
            
            # Evita divisioni per zero esattamente alla gola
            if abs(A_ratio - 1.0) < 1e-4:
                M[i] = 1.0
            else:
                M[i] = self.get_mach_from_area(A_ratio, self.gamma, is_supersonic)
                
            # Relazioni isentropiche per T e P
            temp_ratio = 1.0 + 0.5 * (self.gamma - 1.0) * M[i]**2
            T_gas[i] = T_c / temp_ratio
            P_gas[i] = P_c_pa / (temp_ratio ** (self.gamma / (self.gamma - 1.0)))
            
            # 2. Equazione di Bartz (Semplificata, trascura il raggio di curvatura della gola)
            # h_g = [ 0.026 / (D_t^0.2) ] * [ (mu^0.2 * cp) / Pr^0.6 ] * [ (P_c * g / c*)^0.8 ] * (A_t / A)^0.9 * sigma
            # Assumiamo sigma (fattore di correzione per differenze di proprietà nello strato limite) ~ 1.0
            
            term1 = 0.026 / (D_throat ** 0.2)
            term2 = ((self.mu ** 0.2) * self.cp_gas) / (self.Pr ** 0.6)
            term3 = (P_c_pa / c_star) ** 0.8  # In SI, P_c in Pascal, c* in m/s (g0 è implicito nelle unità metriche per come è definito il c*)
            term4 = (A_throat / A[i]) ** 0.9
            
            h_g[i] = term1 * term2 * term3 * term4
            
            # 3. Modello di Resistenza Termica in Serie
            # R_tot = R_conv_gas + R_cond_wall + R_conv_coolant
            R_gas = 1.0 / h_g[i]
            R_wall = self.t_wall / self.k_wall
            R_cool = 1.0 / h_coolant
            
            q_flux[i] = (T_gas[i] - T_coolant) / (R_gas + R_wall + R_cool)
            
            # 4. Temperature della parete
            T_wall_hot[i] = T_gas[i] - q_flux[i] * R_gas
            T_wall_cold[i] = T_wall_hot[i] - q_flux[i] * R_wall
            
        return x, r, M, T_gas, P_gas, h_g, q_flux, T_wall_hot, T_wall_cold

if __name__ == "__main__":
    solver = SpacialNozzleModel()
    
    # Parametri operativi dal tuo simulatore (MAIN STAGE)
    P_camera = 200e5  # 200 Bar in Pascal
    T_camera = 3600.0 # Temperatura di fiamma [K]
    c_star_eff = 1750.0 # [m/s]
    h_cool = 25000.0  # Coefficiente di scambio del metano nei canali [W/m^2*K]
    T_cool = 120.0    # Temperatura metano liquido [K]
    
    # Calcola il profilo
    x, r, M, T_gas, P_gas, h_g, q, T_w_hot, T_w_cold = solver.compute_1d_profile(
        P_camera, T_camera, c_star_eff, h_cool, T_cool
    )
    
    # ==========================================
    # PLOTTING DEI RISULTATI
    # ==========================================
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # 1. Geometria e Mach
    ax1 = axs[0]
    ax1.plot(x, r, 'k-', lw=2, label='Parete Ugello')
    ax1.plot(x, -r, 'k-', lw=2)
    ax1.fill_between(x, r, -r, color='lightgray', alpha=0.3)
    ax1.set_ylabel('Raggio [m]')
    ax1.set_title('Profilo Motore e Numero di Mach')
    
    ax1_m = ax1.twinx()
    ax1_m.plot(x, M, 'b--', lw=2, label='Mach')
    ax1_m.set_ylabel('Mach [-]', color='b')
    ax1_m.tick_params(axis='y', labelcolor='b')
    ax1.grid(True)
    
    # 2. Temperature Gas e Parete
    ax2 = axs[1]
    ax2.plot(x, T_gas, 'r-', lw=2, label='T_gas')
    ax2.plot(x, T_w_hot, 'orange', lw=2, label='T_parete_lato_gas')
    ax2.plot(x, T_w_cold, 'c-', lw=2, label='T_parete_lato_coolant')
    ax2.axhline(y=1000.0, color='r', linestyle=':', label='Limite Fusione Rame (1000 K)')
    ax2.set_ylabel('Temperatura [K]')
    ax2.set_title('Profilo Termico')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Flusso di Calore e Coefficiente Bartz
    ax3 = axs[2]
    ax3.plot(x, q / 1e6, 'm-', lw=2, label='Flusso di Calore (MW/m^2)')
    ax3.set_ylabel('Flusso Termico [MW/m^2]')
    ax3.set_xlabel('Coordinata Assiale x [m]')
    
    ax3_h = ax3.twinx()
    ax3_h.plot(x, h_g, 'g--', lw=1.5, label='h_gas (Bartz)')
    ax3_h.set_ylabel('Coefficiente di Scambio Gas h_g [W/m^2*K]', color='g')
    ax3_h.tick_params(axis='y', labelcolor='g')
    
    ax3.grid(True)
    plt.tight_layout()
    plt.show()