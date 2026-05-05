"""
structures.py
Modulo di monitoraggio strutturale FFSCC.
Calcola il Margine di Sicurezza (MoS) per i componenti critici.

  MoS = (Carico_Ammissibile / Sforzo_Reale) - 1
  MoS > 0  → sicuro
  MoS = 0  → limite strutturale
  MoS < 0  → ROTTURA
"""
import numpy as np


class StructuralAnalyzer:
    """
    Analizzatore strutturale per il motore FFSCC.

    Metodi principali:
      chamber_wall(p_mcc_bar, t_wall_K)    → MoS parete camera di combustione
      turbine_rotor(rpm, t_K, label)        → MoS disco turbina (centrifugo + termico)
      preburner_vessel(p_bar, label)         → MoS involucro pre-burner
      nozzle_throat(q_flux, t_wall_K)        → MoS gola ugello (fatica termica)
    """

    # ── Materiali ──────────────────────────────────────────────────────────────
    # Rame (Cu-Cr-Zr): parete interna camera + ugello
    _CuCrZr_sy0   = 450e6    # Pa  — snervamento a 300 K
    _CuCrZr_sy_T  = lambda self, T: self._CuCrZr_sy0 * max(0.05, 1.0 - (T - 300.0) / 1800.0)
    _CuCrZr_su    = 550e6    # Pa  — rottura a 300 K

    # Inconel 718: camicia esterna, involucri pre-burner
    _IN718_sy0    = 1100e6   # Pa
    _IN718_sy_T   = lambda self, T: self._IN718_sy0 * max(0.15, 1.0 - (T - 300.0) / 1400.0)
    _IN718_su     = 1380e6   # Pa

    # Titanio Ti-6Al-4V: dischi turbina
    _Ti64_sy0     = 880e6    # Pa
    _Ti64_sy_T    = lambda self, T: self._Ti64_sy0  * max(0.10, 1.0 - (T - 300.0) / 900.0)
    _Ti64_rho     = 4430.0   # kg/m³

    # ── Geometria camera di combustione ───────────────────────────────────────
    _r_mcc        = 0.15     # m  — raggio interno camera
    _t_mcc_cu     = 0.003    # m  — spessore rame
    _t_mcc_in718  = 0.005    # m  — spessore Inconel 718 (camicia esterna)

    # ── Geometria disco turbina ───────────────────────────────────────────────
    _r_disk_ox    = 0.085    # m  — raggio esterno disco turbopompa LOX
    _r_disk_f     = 0.080    # m  — raggio esterno disco turbopompa CH4
    _r_bore       = 0.015    # m  — raggio foro albero

    # ── Geometria involucri pre-burner ─────────────────────────────────────────
    _r_pb         = 0.055    # m  — raggio interno pre-burner
    _t_pb         = 0.008    # m  — spessore parete

    # ── Fattori sicurezza (FS) — Aerospace standard (MIL-HDBK-5) ─────────────
    FS_YIELD  = 1.25
    FS_RUPT   = 1.50

    # ── Fatica termica: limite fluttuazione ΔT per rame ───────────────────────
    _CuCrZr_dT_allow = 600.0  # K  — range ΔT ammissibile per ciclo

    # ── Soglie semaforo ────────────────────────────────────────────────────────
    LIMIT_RED    = 0.0    # MoS < 0   → rosso (rottura)
    LIMIT_YELLOW = 0.25   # MoS < 0.25 → giallo (attenzione)

    # ─────────────────────────────────────────────────────────────────────────

    def chamber_wall_profile(self,
                              p_mcc_bar: float,
                              p_cool_bar: float,
                              x_arr,
                              r_arr,
                              t_cu_profile,
                              t_jacket,
                              T_hw_gas,
                              T_hw_cool,
                              M_arr,
                              gamma: float = 1.14) -> dict:
        """
        Verifica la struttura parete lungo l'intero profilo camera+ugello.

        Fisica della parete composita (da interno a esterno):
          1. Liner in CuCrZr (t_cu):
               Carico netto = Δp = p_gas_locale - p_cool
               Se Δp < 0 (p_cool > p_gas, tipico a regime) → liner in compressione → MoS ∞
               Se Δp > 0 → σ_cu = Δp × r / t_cu  (worst case: bootstrap, p_cool basso)

          2. Camicia strutturale Inconel 718 (t_jacket):
               Regge la pressione del refrigerante come vessel:
               σ_in718 = p_cool × (r + t_cu) / t_jacket
               Temperatura parete ≈ T_cool + ΔT_jacket (assunto 50 K sopra il refrigerante)

        Ritorna il MoS minimo sul profilo con dettaglio del punto critico.
        """
        x_arr    = np.asarray(x_arr,       dtype=float)
        r_arr    = np.asarray(r_arr,        dtype=float)
        t_cu     = np.asarray(t_cu_profile, dtype=float)
        T_gas    = np.asarray(T_hw_gas,     dtype=float)
        T_cool   = np.asarray(T_hw_cool,    dtype=float)
        M_arr    = np.asarray(M_arr,        dtype=float)

        # ── Pressione gas statica locale (isoentropica) ────────────────────────
        exp = gamma / (gamma - 1.0)
        p_gas_pa  = p_mcc_bar * 1e5 / (1.0 + 0.5 * (gamma - 1.0) * M_arr**2) ** exp
        p_cool_pa = p_cool_bar * 1e5

        # ── Liner rame: carico netto gas-cool ──────────────────────────────────
        dp_cu     = p_gas_pa - p_cool_pa                          # positivo se gas > cool
        sigma_cu  = np.maximum(dp_cu, 0.0) * r_arr / np.maximum(t_cu, 1e-6)
        sy_cu_arr = np.array([self._CuCrZr_sy_T(T) for T in T_gas])
        # MoS liner: se in compressione poniamo MoS=10 (non governante)
        mos_cu = np.where(
            dp_cu <= 0.0,
            10.0,
            (sy_cu_arr / self.FS_YIELD) / np.maximum(sigma_cu, 1.0) - 1.0
        )

        # ── Camicia Inconel 718: regge la pressione differenziale ─────────────
        # Il raffreddamento copre tutto il profilo ma la pressione del refrigerante
        # decresce lungo i canali. Stima conservativa: p_cool lineare da p_cool_bar
        # alla gola (M=1) fino a p_gas_locale alle estremità (ΔP → 0 senza flow).
        # In pratica: p_cool_effective = p_gas + (p_cool - p_gas) × decay
        # dove decay = exp(-distance_from_throat / L_ref)
        
        # FIX: Calcolo corretto dell'indice della gola
        i_throat_arr = int(np.argmin(np.abs(M_arr - 1.0)))
        
        dist_from_throat = np.abs(np.arange(len(x_arr)) - i_throat_arr)
        L_ref    = max(len(x_arr) * 0.35, 1.0)                    # ~35% della lunghezza totale
        decay    = np.exp(-dist_from_throat / L_ref)
        p_cool_local = p_gas_pa + (p_cool_pa - p_gas_pa) * decay  # [Pa] lungo il profilo

        r_jacket  = r_arr + t_cu                                   # raggio interno camicia
        sigma_in  = p_cool_local * r_jacket / max(float(t_jacket), 1e-6)
        T_jkt_arr = T_cool + 50.0                                  # stima conservativa
        sy_in_arr = np.array([self._IN718_sy_T(T) for T in T_jkt_arr])
        mos_in    = (sy_in_arr / self.FS_YIELD) / np.maximum(sigma_in, 1.0) - 1.0

        # ── Punto critico: minimo tra i due meccanismi ─────────────────────────
        mos_combined = np.minimum(mos_cu, mos_in)
        i_crit = int(np.argmin(mos_combined))
        mos    = float(mos_combined[i_crit])
        x_c    = float(x_arr[i_crit])

        # Capisce quale meccanismo governa
        if mos_cu[i_crit] < mos_in[i_crit]:
            gov_label = 'liner Cu'
            s_c  = float(sigma_cu[i_crit])
            sy_c = float(sy_cu_arr[i_crit])
            T_c  = float(T_gas[i_crit])
        else:
            gov_label = 'camicia IN718'
            s_c  = float(sigma_in[i_crit])
            sy_c = float(sy_in_arr[i_crit])
            T_c  = float(T_jkt_arr[i_crit])

        zone = 'camera' if x_c < 0.0 else ('gola' if abs(x_c) < 0.02 else 'ugello')

        return {
            'label':     'Parete Cu+IN718 — profilo',
            'sigma_MPa': s_c / 1e6,
            'sy_MPa':    sy_c / 1e6,
            'mos':       mos,
            'detail':    (f'x={x_c:.3f} m ({zone})  [{gov_label}]  '
                          f'σ={s_c/1e6:.0f} MPa  Sy={sy_c/1e6:.0f} MPa  T={T_c:.0f} K'),
            'x_crit':    x_c,
            'mos_arr':   mos_combined,
            'x_arr':     x_arr,
        }

    def turbine_rotor(self, rpm: float, t_K: float, label: str = 'Turbina',
                      is_ox: bool = True) -> dict:
        """
        Verifica disco turbina: sforzo centrifugo (Timoshenko, disco uniforme).
          σ_max = ρ × ω² × (3+ν)/8 × (R² + r²)
          con ν = 0.33 (Ti-6Al-4V)
        """
        omega = rpm * 2.0 * np.pi / 60.0
        nu    = 0.33
        rho   = self._Ti64_rho
        R     = self._r_disk_ox if is_ox else self._r_disk_f
        r     = self._r_bore
        sigma_cf = rho * omega**2 * (3.0 + nu) / 8.0 * (R**2 + r**2)

        sy   = self._Ti64_sy_T(t_K)
        mos  = (sy / self.FS_YIELD) / max(sigma_cf, 1.0) - 1.0

        return {
            'label':     label,
            'sigma_MPa': sigma_cf / 1e6,
            'sy_MPa':    sy / 1e6,
            'mos':       mos,
            'detail':    f'σ_cf={sigma_cf/1e6:.0f} MPa  Sy={sy/1e6:.0f} MPa  T={t_K:.0f} K  {rpm:.0f} RPM',
        }

    def preburner_vessel(self, p_bar: float, label: str = 'Pre-burner') -> dict:
        """
        Verifica involucro sfera pre-burner in Inconel 718.
        Hoop stress sfera: σ = p × r / (2 × t)
        """
        p_pa   = p_bar * 1e5
        sigma  = p_pa * self._r_pb / (2.0 * self._t_pb)

        # Temperatura Inconel pre-burner ≈ 900 K (parete raffreddata a film)
        t_wall = 900.0
        sy     = self._IN718_sy_T(t_wall)
        su     = self._IN718_su

        mos_yield = (sy / self.FS_YIELD) / max(sigma, 1.0) - 1.0
        mos_rupt  = (su / self.FS_RUPT)  / max(sigma, 1.0) - 1.0
        mos       = min(mos_yield, mos_rupt)

        return {
            'label':     label,
            'sigma_MPa': sigma / 1e6,
            'sy_MPa':    sy / 1e6,
            'mos':       mos,
            'detail':    f'σ={sigma/1e6:.0f} MPa  Sy={sy/1e6:.0f} MPa  P={p_bar:.0f} bar',
        }

    def nozzle_throat(self, t_wall_gas_K: float, t_wall_cool_K: float) -> dict:
        """
        Verifica fatica termica della parete rame in gola.
        Criterio semplificato: ΔT attraverso la parete vs ΔT_ammissibile.
        """
        dT_actual = max(t_wall_gas_K - t_wall_cool_K, 0.0)
        mos = self._CuCrZr_dT_allow / max(dT_actual, 1.0) - 1.0

        # Sforzo termico indicativo: σ_th = E × α × ΔT / (2(1-ν))
        E_cu = 110e9; alpha_cu = 17e-6; nu_cu = 0.34
        sigma_th = E_cu * alpha_cu * dT_actual / (2.0 * (1.0 - nu_cu))
        sy_cu    = self._CuCrZr_sy_T(t_wall_gas_K)

        return {
            'label':     'Gola Ugello (fatica termica)',
            'sigma_MPa': sigma_th / 1e6,
            'sy_MPa':    sy_cu / 1e6,
            'mos':       mos,
            'detail':    f'ΔT={dT_actual:.0f} K  σ_th={sigma_th/1e6:.0f} MPa  ΔT_allow={self._CuCrZr_dT_allow:.0f} K',
        }

    @staticmethod
    def mos_color(mos: float) -> str:
        """Restituisce il colore del semaforo per il MoS dato."""
        if mos < StructuralAnalyzer.LIMIT_RED:
            return '#e74c3c'   # rosso
        elif mos < StructuralAnalyzer.LIMIT_YELLOW:
            return '#f39c12'   # giallo
        else:
            return '#2ecc71'   # verde

    @staticmethod
    def mos_label(mos: float) -> str:
        if mos < StructuralAnalyzer.LIMIT_RED:
            return 'FAILURE'
        elif mos < StructuralAnalyzer.LIMIT_YELLOW:
            return 'CAUTION'
        else:
            return 'OK'
