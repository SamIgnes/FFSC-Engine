"""
thermodynamics.py
Proprietà termodinamiche dei propellenti (CH4/LOX) e termochimica CEA.
"""
import numpy as np

# ==========================================
# COOLPROP – GAS REALE CH4 SUPERCRITICO
# ==========================================
try:
    import CoolProp.CoolProp as CP
    _COOLPROP_OK = True
except ImportError:
    CP = None
    _COOLPROP_OK = False
    print("[INFO] CoolProp non trovato – proprietà CH4 semplificate attive.")


class CH4RealGasProps:
    """
    Proprietà termodinamiche e di trasporto del metano (CH4) via CoolProp.

    Punto critico CH4: T_c = 190.564 K, P_c = 45.992 bar.
    In un motore FFSCC il metano scorre a ~100-400 bar, quindi in condizioni
    di liquido denso (T < T_c, P >> P_c) o supercritico (T > T_c, P > P_c).
    """
    T_CRIT     = 190.564   # K
    P_CRIT_BAR = 45.992    # bar

    @staticmethod
    def _q(prop: str, T_K: float, P_bar: float):
        """Singola query CoolProp; ritorna None se fallisce."""
        try:
            T = float(np.clip(T_K, 91.0, 624.0))
            P = float(np.clip(P_bar * 1e5, 1e4, 7e7))
            v = float(CP.PropsSI(prop, 'T', T, 'P', P, 'Methane'))
            return v if np.isfinite(v) else None
        except Exception:
            return None

    @classmethod
    def cp(cls, T_K: float, P_bar: float = 200.0) -> float:
        """Cp reale CH4 [J/kg/K]. Fallback: modello gaussiano semplificato."""
        if _COOLPROP_OK:
            v = cls._q('C', T_K, P_bar)
            if v is not None and 100.0 < v < 5e5:
                return v
        baseline = 3200.0
        peak   = 8500.0 * np.exp(-0.5 * ((T_K - 200.0) / 25.0)**2)
        cp_lat = (511000.0 / (4.0 * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((T_K - 111.7) / 4.0)**2)
        return baseline + peak + cp_lat

    @classmethod
    def density(cls, T_K: float, P_bar: float = 200.0) -> float:
        if _COOLPROP_OK:
            v = cls._q('D', T_K, P_bar)
            if v is not None and v > 0:
                return v
        return 422.0 if T_K < cls.T_CRIT else 160.0

    @classmethod
    def viscosity(cls, T_K: float, P_bar: float = 200.0) -> float:
        if _COOLPROP_OK:
            v = cls._q('V', T_K, P_bar)
            if v is not None and v > 0:
                return v
        return 1.5e-4

    @classmethod
    def thermal_conductivity(cls, T_K: float, P_bar: float = 200.0) -> float:
        if _COOLPROP_OK:
            v = cls._q('L', T_K, P_bar)
            if v is not None and v > 0:
                return v
        return 0.18

    @classmethod
    def vapor_pressure_bar(cls, T_K: float) -> float:
        """Pressione di saturazione CH4 [bar]. Sopra T_c ritorna P_c."""
        if T_K >= cls.T_CRIT:
            return cls.P_CRIT_BAR
        if _COOLPROP_OK:
            try:
                T = float(np.clip(T_K, 91.0, cls.T_CRIT - 0.1))
                v = float(CP.PropsSI('P', 'T', T, 'Q', 0, 'Methane'))
                if np.isfinite(v) and v > 0:
                    return v / 1e5
            except Exception:
                pass
        if T_K < 90:
            return 0.1
        return 10**(3.98 - 443.0 / (T_K - 0.5))

    @classmethod
    def phase_label(cls, T_K: float, P_bar: float) -> str:
        if P_bar > cls.P_CRIT_BAR and T_K > cls.T_CRIT:
            return "SUPERCRIT"
        elif P_bar > cls.P_CRIT_BAR:
            return "DENSE-LIQ"
        elif T_K > cls.T_CRIT:
            return "GAS"
        else:
            return "LIQ"

    @classmethod
    def cp_array(cls, T_arr: np.ndarray, P_bar: float = 200.0) -> np.ndarray:
        """Cp su tutto l'array in una sola chiamata CoolProp (vectorized)."""
        if not _COOLPROP_OK:
            return np.vectorize(lambda T: cls.cp(T, P_bar))(T_arr)
        T_clip = np.clip(T_arr.astype(float), 91.0, 624.0)
        P_arr  = np.full_like(T_clip, float(np.clip(P_bar * 1e5, 1e4, 7e7)))
        try:
            vals = CP.PropsSI('C', 'T', T_clip, 'P', P_arr, 'Methane')
            vals = np.where(np.isfinite(vals) & (vals > 100) & (vals < 5e5), vals,
                            np.vectorize(lambda T: cls.cp(T, P_bar))(T_clip))
            return vals
        except Exception:
            return np.vectorize(lambda T: cls.cp(T, P_bar))(T_clip)


class MethaloxProperties:
    @staticmethod
    def get_ch4_cp(T_coolant, P_bar: float = 200.0):
        """Cp CH4 [J/kg/K] – usa CoolProp se disponibile."""
        return CH4RealGasProps.cp(T_coolant, P_bar)


def _load_cea_3d():
    """
    Carica la tabella CEA 3D (OF × T_fuel_in) se disponibile.
    Ritorna (interpolatori, of_min, of_max, tf_min, tf_max) oppure None.
    """
    import os
    path = os.path.join(os.path.dirname(__file__), 'cea_table_ch4_lox_3d.npz')
    if not os.path.exists(path):
        return None
    try:
        from scipy.interpolate import RegularGridInterpolator
        d = np.load(path)
        of_pts = d['of_points']
        tf_pts = d['t_fuel_points']
        interps = {}
        for key in ('t_ad', 'c_star', 'gamma', 'mw'):
            interps[key] = RegularGridInterpolator(
                (of_pts, tf_pts), d[key],
                method='linear', bounds_error=False, fill_value=None
            )
        print(f"[CEA 3D] Tabella {len(of_pts)}×{len(tf_pts)} caricata da {path}")
        return interps, float(of_pts[0]), float(of_pts[-1]), float(tf_pts[0]), float(tf_pts[-1])
    except Exception as e:
        print(f"[CEA 3D] Errore caricamento: {e} — fallback a tabella 2D")
        return None


class CEA_MethaloxCombustion:
    """
    Termochimica CH4/LOX tabulata da dati CEA (NASA-SP-273 + calcoli NIST).
    Copre O/F da 0.15 (fuel-rich estremo) a 50 (quasi puro ossigeno).
    Riferimento: P = 200 bar. Correzione logaritmica per altre pressioni.

    Se presente 'cea_table_ch4_lox_3d.npz' (generato da generate_cea_table.py),
    usa interpolazione 3D RegularGridInterpolator su (OF, T_fuel_in).
    Altrimenti usa la tabella 2D hardcoded + correzione entalpica analitica.

    Colonne tabella 2D: [O/F, T_ad(K), c_star(m/s), gamma, MW(g/mol)]
    """
    _TABLE = np.array([
        [   0.15,    612.2,   844.1, 1.1824, 16.005],
        [   0.20,    747.5,   938.2, 1.1569, 15.751],
        [   0.30,    931.4,  1060.8, 1.1412, 14.913],
        [   0.40,   1051.1,  1145.5, 1.1369, 14.277],
        [   0.60,   1199.0,  1261.5, 1.1398, 13.932],
        [   0.80,   1293.5,  1340.2, 1.1507, 14.201],
        [   1.00,   1373.5,  1398.8, 1.1837, 14.061],
        [   1.50,   1702.6,  1552.7, 1.2642, 13.524],
        [   2.00,   2558.3,  1757.4, 1.2319, 16.030],
        [   2.50,   3209.7,  1856.5, 1.1830, 18.549],
        [   3.00,   3578.4,  1879.7, 1.1478, 20.668],
        [   3.40,   3689.2,  1859.7, 1.1359, 21.966],
        [   3.60,   3707.2,  1843.4, 1.1335, 22.505],
        [   4.00,   3705.0,  1807.4, 1.1315, 23.429],
        [   5.00,   3611.2,  1720.8, 1.1320, 25.175],
        [   6.00,   3480.4,  1646.6, 1.1356, 26.425],
        [   8.00,   3198.3,  1524.5, 1.1485, 28.056],
        [  10.00,   2918.9,  1422.9, 1.1671, 29.002],
        [  15.00,   2314.2,  1225.8, 1.2146, 30.085],
        [  20.00,   1879.3,  1087.4, 1.2439, 30.547],
        [  30.00,   1347.9,   906.6, 1.2739, 31.004],
        [  40.00,   1042.0,   789.6, 1.2933, 31.241],
        [  50.00,    842.2,   704.8, 1.3102, 31.387],
    ])
    _OF    = _TABLE[:, 0]
    _T_AD  = _TABLE[:, 1]
    _CSTAR = _TABLE[:, 2]
    _GAMMA = _TABLE[:, 3]
    _MW    = _TABLE[:, 4]

    # Temperatura di riferimento della tabella 2D hardcoded [K]
    _T_FUEL_REF = 112.0

    # Caricamento tabella 3D a import-time (None se non disponibile)
    _3D = _load_cea_3d()

    # Valori di ancoraggio all'estremo INFERIORE della tabella (OF=0.15)
    _OF_MIN    = 0.15
    _TAD_015   = 612.2
    _CSTAR_015 = 844.1
    _GAMMA_015 = 1.1824
    _MW_015    = 16.005
    # Temperatura fuel di riferimento per il limite basso (stessa della tabella CEA 2D)
    _T_FUEL_REF_LOW = 112.0  # K

    # Valori di ancoraggio all'estremo superiore della tabella (OF=50)
    _OF_MAX    = 50.0
    _TAD_50    = 842.2
    _CSTAR_50  = 704.8
    _GAMMA_50  = 1.3102
    _MW_50     = 31.387
    # Temperatura ingresso ossidante [K] per il bilancio energetico
    _T_OX_IN   = 92.0

    @classmethod
    def _extrapolate_low_of(cls, of_ratio, t_fuel_k=None):
        """
        Estrapolazione analitica per OF < 0.15 (regime fuel-rich estremo, quasi puro CH4).

        Fisica: con pochissimo ossidante la combustione è incompleta.
        T_ad scala linearmente verso T_fuel puro (nessun calore rilasciato a OF=0).
        MW → 16 g/mol (CH4 puro), gamma → 1.31 (CH4).
        """
        of = max(0.0, float(of_ratio))
        t_fuel = t_fuel_k if t_fuel_k is not None else cls._T_FUEL_REF_LOW
        alpha = of / cls._OF_MIN   # 0 a OF=0, 1 a OF=0.15

        t_ad  = t_fuel + (cls._TAD_015 - t_fuel) * alpha
        gamma = 1.31 + (cls._GAMMA_015 - 1.31) * alpha
        mw    = 16.0 + (cls._MW_015 - 16.0) * alpha
        # c_star → 0 a OF=0 (nessun propellente ossidante), scala linearmente
        cstar = cls._CSTAR_015 * alpha
        return t_ad, cstar, gamma, mw

    @classmethod
    def _extrapolate_high_of(cls, of_ratio, t_fuel_k=None):
        """
        Estrapolazione analitica per OF > 50 (regime ox-rich estremo).

        Fisica: l'eccesso di O2 agisce da pozzo di calore.
        Bilancio energetico semplificato (calibrato su OF=50):
          T_ad(OF) = T_ox + (T_ad_50 - T_ox) * (1 + OF_MAX) / (1 + OF)
        c_star scala con sqrt(T_ad / MW) (approssimazione isoentropica).
        gamma -> 1.40 (O2 biatomico), MW -> 32.0 (puro O2).
        """
        of = max(float(of_ratio), cls._OF_MAX)
        alpha = (1.0 + cls._OF_MAX) / (1.0 + of)   # fattore di diluizione

        t_ad  = cls._T_OX_IN + (cls._TAD_50 - cls._T_OX_IN) * alpha
        gamma = 1.40 - (1.40 - cls._GAMMA_50) * alpha
        mw    = 32.0 - (32.0 - cls._MW_50) * alpha
        # c_star proporzionale a sqrt(T_ad / MW) — normalizzato al punto di ancoraggio
        cstar = cls._CSTAR_50 * np.sqrt((t_ad / cls._TAD_50) * (cls._MW_50 / mw))
        return t_ad, cstar, gamma, mw

    @classmethod
    def _interp(cls, of_ratio, col):
        of_c = np.clip(float(of_ratio), cls._OF[0], cls._OF[-1])
        return float(np.interp(of_c, cls._OF, col))

    @classmethod
    def _query_3d(cls, key, of_ratio, t_fuel):
        """Interroga la tabella 3D se disponibile. Ritorna None altrimenti."""
        if cls._3D is None:
            return None
        interps, of_min, of_max, tf_min, tf_max = cls._3D
        of_c = np.clip(float(of_ratio), of_min, of_max)
        tf_c = np.clip(float(t_fuel),   tf_min, tf_max)
        return float(interps[key]([[of_c, tf_c]])[0])

    @classmethod
    def _t_fuel_correction(cls, of_ratio, t_fuel_k, t_ad_2d):
        """
        Correzione entalpica analitica a T_ad quando la tabella 3D non è disponibile.
        Modella il pre-riscaldamento del CH4 nel cooling jacket prima di entrare in FRHC.

        Bilancio entalpico semplificato: la variazione di temperatura adiabatica
        è proporzionale alla variazione di entalpia del carburante rispetto alla
        temperatura di riferimento della tabella, pesata sulla frazione massica fuel.

            ΔT_ad ≈ (T_fuel_in - T_ref) / (1 + OF)

        Questo segue dal bilancio: ΔH_fuel = mdot_f * cp_f * ΔT_fuel,
        e i cp_f e cp_products si approssimano dello stesso ordine, per cui
        si cancellano lasciando solo la frazione massica 1/(1+OF).
        """
        delta_t_fuel = t_fuel_k - cls._T_FUEL_REF
        if abs(delta_t_fuel) < 1.0:
            return t_ad_2d
        of_c = max(float(of_ratio), 0.01)
        correction = delta_t_fuel / (1.0 + of_c)
        return t_ad_2d + correction

    @classmethod
    def get_t_ad(cls, of_ratio, p_bar=200.0, t_fuel=None):
        """
        Temperatura adiabatica di fiamma [K].

        t_fuel: temperatura ingresso CH4 [K]. Se None usa il valore di riferimento
                della tabella 2D (112 K). Rilevante per il FRHC pre-burner dove
                il CH4 entra già riscaldato dal cooling jacket (~300-450 K).
        """
        p_corr = 1.0 + 0.015 * np.log10(np.clip(p_bar, 0.1, 1000.0) / 200.0)
        t_fuel_k = float(t_fuel) if t_fuel is not None else cls._T_FUEL_REF

        # Estrapolazione analitica fuori dal range CEA
        if float(of_ratio) > cls._OF_MAX:
            t_ad, _, _, _ = cls._extrapolate_high_of(of_ratio, t_fuel_k)
            return t_ad * p_corr
        if float(of_ratio) < cls._OF_MIN:
            t_ad, _, _, _ = cls._extrapolate_low_of(of_ratio, t_fuel_k)
            return t_ad * p_corr

        # Tabella 3D disponibile → interpolazione diretta
        v3d = cls._query_3d('t_ad', of_ratio, t_fuel_k)
        if v3d is not None:
            return v3d * p_corr

        # Fallback: tabella 2D + correzione entalpica analitica
        t = cls._interp(of_ratio, cls._T_AD)
        t = cls._t_fuel_correction(of_ratio, t_fuel_k, t)
        return t * p_corr

    @classmethod
    def get_c_star(cls, of_ratio, p_bar=200.0, t_fuel=None):
        """
        Velocità caratteristica ideale c* [m/s].

        t_fuel: temperatura ingresso CH4 [K]. Effetto su c* più debole rispetto
                a T_ad, ma comunque modellato nella tabella 3D.
        """
        p_corr = 1.0 + 0.008 * np.log10(np.clip(p_bar, 0.1, 1000.0) / 200.0)
        t_fuel_k = float(t_fuel) if t_fuel is not None else cls._T_FUEL_REF

        # Estrapolazione analitica fuori dal range CEA
        if float(of_ratio) > cls._OF_MAX:
            _, cstar, _, _ = cls._extrapolate_high_of(of_ratio, t_fuel_k)
            return cstar * p_corr
        if float(of_ratio) < cls._OF_MIN:
            _, cstar, _, _ = cls._extrapolate_low_of(of_ratio, t_fuel_k)
            return cstar * p_corr

        v3d = cls._query_3d('c_star', of_ratio, t_fuel_k)
        if v3d is not None:
            return v3d * p_corr

        return cls._interp(of_ratio, cls._CSTAR) * p_corr

    @classmethod
    def get_gamma(cls, of_ratio, t_fuel=None):
        """Rapporto dei calori specifici gamma [-]."""
        t_fuel_k = float(t_fuel) if t_fuel is not None else cls._T_FUEL_REF
        # Estrapolazione analitica fuori dal range CEA
        if float(of_ratio) > cls._OF_MAX:
            _, _, gamma, _ = cls._extrapolate_high_of(of_ratio, t_fuel_k)
            return gamma
        if float(of_ratio) < cls._OF_MIN:
            _, _, gamma, _ = cls._extrapolate_low_of(of_ratio, t_fuel_k)
            return gamma

        v3d = cls._query_3d('gamma', of_ratio, t_fuel_k)
        if v3d is not None:
            return v3d
        return cls._interp(of_ratio, cls._GAMMA)

    @classmethod
    def get_mw(cls, of_ratio, t_fuel=None):
        """Massa molecolare media dei prodotti di combustione [g/mol]."""
        t_fuel_k = float(t_fuel) if t_fuel is not None else cls._T_FUEL_REF
        # Estrapolazione analitica fuori dal range CEA
        if float(of_ratio) > cls._OF_MAX:
            _, _, _, mw = cls._extrapolate_high_of(of_ratio, t_fuel_k)
            return mw
        if float(of_ratio) < cls._OF_MIN:
            _, _, _, mw = cls._extrapolate_low_of(of_ratio, t_fuel_k)
            return mw

        v3d = cls._query_3d('mw', of_ratio, t_fuel_k)
        if v3d is not None:
            return v3d
        return cls._interp(of_ratio, cls._MW)

    @staticmethod
    def compute_cf(gamma, eps, p_c_pa, p_a_pa=101325.0):
        """
        Coefficiente di spinta Cf da teoria ugello isoentropico.
        eps = Ae/At. Newton-Raphson sulla relazione area-Mach (ramo supersonico).
        """
        gm1 = gamma - 1.0
        gp1 = gamma + 1.0
        Me = 5.0
        for _ in range(40):
            t  = 1.0 + gm1 / 2.0 * Me**2
            h  = 2.0 * t / gp1
            u  = h ** (gp1 / (2.0 * gm1))
            f  = u / Me - eps
            df = (u / h) * (Me**2 - h) / Me**2
            if abs(df) < 1e-15:
                break
            Me -= f / df
            Me  = max(Me, 1.01)
            if abs(f) < 1e-9:
                break
        p_e    = p_c_pa / (1.0 + gm1 / 2.0 * Me**2) ** (gamma / gm1)
        cf_vac = np.sqrt(2.0 * gamma**2 / gm1 * (2.0 / gp1)**(gp1 / gm1) *
                         (1.0 - (p_e / p_c_pa)**(gm1 / gamma)))
        cf = cf_vac + (p_e - p_a_pa) / p_c_pa * eps
        return max(0.0, cf)


class PropellantPhysics:
    @staticmethod
    def get_vapor_pressure_bar(T, prop="CH4"):
        if prop == "CH4":
            return CH4RealGasProps.vapor_pressure_bar(T)
        else:
            # LOX – Antoine semplificato
            if T < 70:
                return 0.1
            return 10**(4.08 - 366.0 / (T - 4.5))
