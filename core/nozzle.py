"""
nozzle.py
Modello termico spaziale 1-D dell'ugello (parete in rame, 4 nodi radiali).
"""
import numpy as np
from scipy.optimize import fsolve
from core.thermodynamics import CH4RealGasProps, CEA_MethaloxCombustion, _COOLPROP_OK

try:
    import CoolProp.CoolProp as CP
except ImportError:
    CP = None


class SpacialNozzleModel:
    def __init__(self):
        # Proprietà gas di combustione
        self.gamma   = 1.14
        self.Pr      = 0.7
        self.mu      = 1.2e-4
        self.cp_gas  = 2800.0

        # Geometria
        self.x, self.r = self._generate_geometry()
        self.A             = np.pi * self.r**2
        self.circumference = 2.0 * np.pi * self.r
        self.A_throat      = np.min(self.A)
        self.D_throat      = 2.0 * np.min(self.r)
        self.dx            = np.abs(self.x[1] - self.x[0])

        # Nodo rame con differenze finite (4 nodi radiali)
        self.k_hw   = 350.0    # W/m/K
        self.rho_hw = 8960.0   # kg/m³
        self.cp_hw  = 385.0    # J/kg/K
        self.N_rad  = 4

        # Profilo di spessore variabile: minimo alla gola (1.0 mm) per ridurre
        # la resistenza di conduzione dove il flusso di calore è massimo.
        # Fisicamente: i motori FFSCC reali (Raptor) usano pareti più sottili alla gola.
        #   t_throat = 1.0 mm → t_hw/k_hw = 2.86e-6 m²K/W  (vs 5.71e-6 con 2 mm)
        #   t_ends   = 2.5 mm → resistenza extra dove il flusso è minore
        i_throat_hw  = int(np.argmin(self.A))
        t_hw_min     = 0.0010   # 1.0 mm alla gola
        t_hw_max     = 0.0025   # 2.5 mm alle estremità
        sigma_nodes  = 12       # larghezza Gaussiana (nodi)
        gauss = np.exp(-((np.arange(len(self.x)) - i_throat_hw)**2) / (2.0 * sigma_nodes**2))
        self.t_hw_profile = t_hw_max - (t_hw_max - t_hw_min) * gauss  # [n] m

        # Passo radiale per nodo (vettore, uno per posizione assiale)
        self.dr_profile = self.t_hw_profile / (self.N_rad - 1)  # [n] m

        # Scalare di compatibilità (usato nei calcoli dt_max)
        self.t_hw = float(np.mean(self.t_hw_profile))
        self.dr   = float(np.mean(self.dr_profile))

        # Matrice 2D: [coord_X, nodo_radiale]  (0=gas, 3=refrigerante)
        self.T_hw_rad = np.full((len(self.x), self.N_rad), 300.0)

        self.C_vol          = self.rho_hw * self.cp_hw
        self.C_surf_profile = self.C_vol * (self.dr_profile / 2.0)  # [n] J/m²/K
        self.C_int_profile  = self.C_vol * self.dr_profile           # [n] J/m²/K
        # Scalari per compatibilità
        self.C_surf = self.C_vol * (self.dr / 2.0)
        self.C_int  = self.C_vol * self.dr

        # Parete esterna in acciaio (1 nodo)
        self.t_cw       = 0.005
        self.k_cw       = 15.0
        self.C_area_cw  = 8000.0 * 500.0 * self.t_cw
        self.epsilon    = 0.8
        self.sigma      = 5.67e-8
        self.T_amb      = 300.0

        self.T_cool = np.full_like(self.x, 120.0)
        self.T_cw   = np.full_like(self.x, 300.0)

        # Geometria canali di raffreddamento
        self.N_ch = 100
        self.w_ch = 0.003
        self.h_ch = 0.006
        self.D_h  = 2.0 * self.w_ch * self.h_ch / (self.w_ch + self.h_ch)
        self.A_ch_total = self.N_ch * self.w_ch * self.h_ch

        # Pre-calcolo Mach isoentropico
        self.M = np.zeros_like(self.x)
        for i, xi in enumerate(self.x):
            A_ratio = self.A[i] / self.A_throat
            if abs(A_ratio - 1.0) < 1e-4:
                self.M[i] = 1.0
            else:
                self.M[i] = self._get_mach(A_ratio, xi > 0.0)
        self.temp_ratio = 1.0 + 0.5 * (self.gamma - 1.0) * self.M**2

    def _generate_geometry(self):
        x = np.linspace(-0.2, 0.6, 100)
        r = np.zeros_like(x)
        for i, xi in enumerate(x):
            if xi < -0.1:
                r[i] = 0.15
            elif xi <= 0.0:
                r[i] = 0.05 + (0.15 - 0.05) * (abs(xi) / 0.1)
            else:
                r[i] = 0.05 + (0.35 - 0.05) * (xi / 0.6)**1.5
        return x, r

    def _get_mach(self, A_ratio, is_supersonic):
        def eq(M):
            if M <= 0:
                return 1e6
            t = (2.0 / (self.gamma + 1.0)) * (1.0 + 0.5 * (self.gamma - 1.0) * M**2)
            return (1.0 / M) * (t**((self.gamma + 1.0) / (2.0 * (self.gamma - 1.0)))) - A_ratio
        return fsolve(eq, 3.0 if is_supersonic else 0.1)[0]

    def _h_cool_nusselt(self, mdot_coolant: float, coolant_p_bar: float) -> np.ndarray:
        """Coefficiente convettivo lato CH4 [W/m²/K] – Dittus-Boelter con CoolProp."""
        T_mean = float(np.mean(self.T_cool))
        if _COOLPROP_OK:
            try:
                T_q  = float(np.clip(T_mean, 91.0, 624.0))
                P_q  = float(np.clip(coolant_p_bar * 1e5, 1e4, 7e7))
                rho_c = float(CP.PropsSI('D', 'T', T_q, 'P', P_q, 'Methane'))
                mu_c  = float(CP.PropsSI('V', 'T', T_q, 'P', P_q, 'Methane'))
                k_c   = float(CP.PropsSI('L', 'T', T_q, 'P', P_q, 'Methane'))
                cp_c  = float(CP.PropsSI('C', 'T', T_q, 'P', P_q, 'Methane'))
                Pr_c  = float(np.clip(mu_c * cp_c / k_c, 0.1, 100.0))

                v_ref  = max(mdot_coolant, 0.1) / max(rho_c * self.A_ch_total, 1e-9)
                Re_ref = rho_c * v_ref * self.D_h / mu_c
                Re_ref = max(Re_ref, 10.0)

                Nu_ref = 0.023 * Re_ref**0.8 * Pr_c**0.4
                h_ref  = Nu_ref * k_c / self.D_h

                v_factor = (self.A[0] / np.maximum(self.A, 1e-9))**0.8
                return np.clip(h_ref * v_factor, 500.0, 5e5)
            except Exception:
                pass

        fin_enhancement = 3.0
        v_factor = (self.A[0] / np.maximum(self.A, 1e-9))**0.8
        return 25000.0 * max(0.01, (mdot_coolant / 50.0)**0.8) * fin_enhancement * v_factor

    def compute_instantaneous_profile(self, P_c_pa, T_c, c_star, mdot_coolant, T_inlet, dt,
                                      coolant_pressure_bar: float = 200.0,
                                      frac_chamber: float = 0.5):
        """
        Raffreddamento bidirezionale: il CH4 entra alla gola (punto più caldo),
        si divide in due flussi paralleli:
          • frac_chamber  × mdot → risale verso la camera di combustione
          • (1-frac_chamber) × mdot → scende verso l'uscita dell'ugello
        """
        n = len(self.x)
        i_throat = int(np.argmin(self.A))   # indice della gola (area minima)

        mdot_up   = mdot_coolant * frac_chamber          # flusso verso camera
        mdot_down = mdot_coolant * (1.0 - frac_chamber)  # flusso verso uscita

        if P_c_pa < 105000:
            h_g   = np.full_like(self.x, 10.0)
            T_gas = np.full_like(self.x, 300.0)
        else:
            T_gas = T_c / self.temp_ratio
            h_g   = (0.026 / self.D_throat**0.2) * ((self.mu**0.2 * self.cp_gas) / self.Pr**0.6) \
                    * (P_c_pa / c_star)**0.8 * (self.A_throat / self.A)**0.9

        # Coefficienti convettivi calcolati per ciascun semi-flusso
        h_up   = self._h_cool_nusselt(mdot_up,   coolant_pressure_bar)
        h_down = self._h_cool_nusselt(mdot_down, coolant_pressure_bar)
        # Merge: sezione camera usa h_up, sezione ugello usa h_down
        idx    = np.arange(n)
        h_cool_array = np.where(idx <= i_throat, h_up, h_down)

        cp_profile = CH4RealGasProps.cp_array(self.T_cool, coolant_pressure_bar)

        q_hw_cool = np.zeros(n)
        q_cool_cw = np.zeros(n)

        R_cw_cond = (self.t_cw / 2.0) / self.k_cw

        # ── Iniezione alla gola ───────────────────────────────────────────────
        self.T_cool[i_throat] = T_inlet

        # ── Flusso verso camera: gola → testa camera (i decrescente) ─────────
        for i in range(i_throat, -1, -1):
            h_local       = h_cool_array[i]
            q_hw_cool[i]  = h_local * (self.T_hw_rad[i, -1] - self.T_cool[i])
            q_cool_cw[i]  = (self.T_cool[i] - self.T_cw[i]) / (1.0 / max(h_local, 1e-6) + R_cw_cond)

            if i > 0:
                cp_local    = float(cp_profile[i])
                Q_net       = (q_hw_cool[i] - q_cool_cw[i]) * (self.circumference[i] * self.dx)
                dT          = Q_net / (max(0.1, mdot_up) * cp_local)
                # Il flusso va verso indici decrescenti: T[i-1] = T[i] + calore assorbito
                self.T_cool[i - 1] = self.T_cool[i] + dT

        # ── Flusso verso uscita ugello: gola → exit (i crescente) ────────────
        self.T_cool[i_throat] = T_inlet  # re-imposta gola (sovrascritto dal loop sopra)
        for i in range(i_throat, n):
            h_local       = h_cool_array[i]
            q_hw_cool[i]  = h_local * (self.T_hw_rad[i, -1] - self.T_cool[i])
            q_cool_cw[i]  = (self.T_cool[i] - self.T_cw[i]) / (1.0 / max(h_local, 1e-6) + R_cw_cond)

            if i < n - 1:
                cp_local    = float(cp_profile[i])
                Q_net       = (q_hw_cool[i] - q_cool_cw[i]) * (self.circumference[i] * self.dx)
                dT          = Q_net / (max(0.1, mdot_down) * cp_local)
                # Il flusso va verso indici crescenti: T[i+1] = T[i] + calore assorbito
                self.T_cool[i + 1] = self.T_cool[i] + dT

        # Il flusso verso camera esce più caldo alla testa (T_cool[0])
        # Il flusso verso ugello esce più caldo all'exit   (T_cool[n-1])
        # La temperatura usata dal CoolingJacket (T_cool medio) rimane coerente

        # Integrazione temporale con sub-stepping (stabilità Fourier + convezione)
        # Stabilità Fourier con dr minimo (gola = nodi più sottili → dt più restrittivo)
        alpha        = self.k_hw / self.C_vol
        dr_min       = float(np.min(self.dr_profile))
        dt_max_cond  = 0.4 * (dr_min**2) / alpha
        h_max        = max(float(np.max(h_g)), float(np.max(h_cool_array)))
        C_surf_min   = float(np.min(self.C_surf_profile))
        k_over_dr_max = float(self.k_hw / dr_min)
        dt_max_conv  = 0.4 * C_surf_min / (k_over_dr_max + h_max)
        dt_max       = min(dt_max_cond, dt_max_conv)
        num_steps    = max(1, int(np.ceil(dt / dt_max)))
        dt_sub       = dt / num_steps

        # Precalcola conduttanze per vettorizzazione (dipendono da dr_profile)
        k_dr = self.k_hw / self.dr_profile   # [n]  W/m²/K per la conduzione

        for _ in range(num_steps):
            # Nodo 0: faccia gas (spessore ½ dr)
            q_in_gas   = h_g * (T_gas - self.T_hw_rad[:, 0])
            q_cond_0_1 = k_dr * (self.T_hw_rad[:, 0] - self.T_hw_rad[:, 1])
            self.T_hw_rad[:, 0] += ((q_in_gas - q_cond_0_1) / self.C_surf_profile) * dt_sub

            # Nodi interni 1, 2
            for j in range(1, self.N_rad - 1):
                q_cond_in  = k_dr * (self.T_hw_rad[:, j - 1] - self.T_hw_rad[:, j])
                q_cond_out = k_dr * (self.T_hw_rad[:, j]     - self.T_hw_rad[:, j + 1])
                self.T_hw_rad[:, j] += ((q_cond_in - q_cond_out) / self.C_int_profile) * dt_sub

            # Nodo 3: faccia refrigerante (spessore ½ dr)
            q_cond_2_3 = k_dr * (self.T_hw_rad[:, 2] - self.T_hw_rad[:, 3])
            q_out_cool = h_cool_array * (self.T_hw_rad[:, 3] - self.T_cool)
            self.T_hw_rad[:, 3] += ((q_cond_2_3 - q_out_cool) / self.C_surf_profile) * dt_sub

        self.T_cool = np.clip(self.T_cool, 90.0, 4000.0)

        # Camicia esterna in acciaio
        q_rad    = self.epsilon * self.sigma * (self.T_cw**4 - self.T_amb**4)
        dT_cw_dt = (q_cool_cw - q_rad) / self.C_area_cw
        self.T_cw += dT_cw_dt * dt

        T_hw_gas_face  = self.T_hw_rad[:, 0]
        T_hw_cool_face = self.T_hw_rad[:, -1]

        return self.x, T_gas, T_hw_gas_face, T_hw_cool_face, self.T_cool, self.T_cw
