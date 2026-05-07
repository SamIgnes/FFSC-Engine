import numpy as np
from config import (
    PID_THRUST_KP, PID_THRUST_KI,
    PID_MIXTURE_KP, PID_MIXTURE_KI,
    PID_ORHC_KP, PID_ORHC_KI,
    PID_FRHC_KP, PID_FRHC_KI,
    PID_TANK_KP, PID_TANK_KI,
    PID_CHILL_KP, PID_CHILL_KI,
    TARGET_T_CHILLDOWN, TARGET_OF_ORHC, TARGET_OF_FRHC, TARGET_OF_MCC, TARGET_P_TANK,
    TRIM_OF_MCC_MAX, TRIM_ORHC_MAX, TRIM_FRHC_MAX,
    VALVE_ORHC_NOMINAL, VALVE_FRHC_NOMINAL,
    ABORT_P_MCC_BAR, ABORT_RPM_LIMIT, ABORT_T_COOL_K, ABORT_P_TANK_BAR,
    SEQ_CHILLDOWN_T_COOL_MAX, SEQ_CHILLDOWN_T_MIN, SEQ_CHILLDOWN_TO,
    SEQ_SPINPRIME_RPM_OX_MIN, SEQ_SPINPRIME_RPM_F_MIN,
    SEQ_SPINPRIME_P_TANK_MIN, SEQ_SPINPRIME_T_MIN, SEQ_SPINPRIME_TO,
    SEQ_BOOTSTRAP_TO, SEQ_IGNITION_RPM_F, SEQ_IGNITION_RPM_OX,
    SEQ_RAMPUP_THRUST, SEQ_RAMPUP_DURATION, THRUST_RATE_KN_S,
    IGN_TH_MOV, IGN_TH_MFV, IGN_V_FRHC, IGN_V_ORHC,
    RAMP_TH_MOV_0, RAMP_TH_MOV_1,
    RAMP_TH_MFV_0, RAMP_TH_MFV_1,
    RAMP_V_FRHC_0, RAMP_V_FRHC_1,
    RAMP_V_ORHC_0, RAMP_V_ORHC_1,
)

class PI_Controller:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0.0

    def compute(self, target, measured):
        error = target - measured
        self.integral += error * self.dt
        max_int = 1.0 / self.ki if self.ki > 0 else 0
        self.integral = max(-max_int, min(max_int, self.integral))
        output = (self.kp * error) + (self.ki * self.integral)
        return max(0.0, min(1.0, output))

    def compute_trim(self, target, measured, max_trim=0.06):
        error = target - measured
        self.integral += error * self.dt
        max_int = max_trim / (self.ki + 1e-9)
        self.integral = max(-max_int, min(max_int, self.integral))
        output = (self.kp * error) + (self.ki * self.integral)
        return max(-max_trim, min(max_trim, output))


class FlightComputer:
    def __init__(self, dt):
        self.dt = dt
        self.state        = "IDLE"
        self.timer        = 0.0
        self.abort_reason = ""

        self.pid_thrust    = PI_Controller(kp=PID_THRUST_KP,  ki=PID_THRUST_KI,  dt=dt)
        self.pid_mixture   = PI_Controller(kp=PID_MIXTURE_KP, ki=PID_MIXTURE_KI, dt=dt)
        self.pid_orhc      = PI_Controller(kp=PID_ORHC_KP,    ki=PID_ORHC_KI,    dt=dt)
        self.pid_frhc      = PI_Controller(kp=PID_FRHC_KP,    ki=PID_FRHC_KI,    dt=dt)
        self.pid_tank_ox   = PI_Controller(kp=PID_TANK_KP,    ki=PID_TANK_KI,    dt=dt)
        self.pid_tank_f    = PI_Controller(kp=PID_TANK_KP,    ki=PID_TANK_KI,    dt=dt)
        self.pid_chilldown = PI_Controller(kp=PID_CHILL_KP,   ki=PID_CHILL_KI,   dt=dt)

        self.target_t_chilldown     = TARGET_T_CHILLDOWN
        self.target_of_orhc         = TARGET_OF_ORHC
        self.target_of_frhc         = TARGET_OF_FRHC
        self.target_p_tank          = TARGET_P_TANK
        self.internal_target_thrust = 0.0
        self.target_of              = TARGET_OF_MCC
        self.of_boost_integral      = 0.0
        self._transition_th_mov     = None
        self._transition_th_mfv     = None
        self._transition_v_orhc     = None
        self._transition_v_frhc     = None
        self._main_stage_hold_steps = 0

    def start_sequence(self):
        if self.state in ["IDLE", "MECO", "ABORT"]:
            self.state, self.timer, self.abort_reason = "CHILLDOWN", 0.0, ""

    def meco(self):
        if self.state not in ["IDLE", "MECO", "ABORT"]:
            self.state, self.timer, self.internal_target_thrust, self.abort_reason = "MECO", 0.0, 0.0, ""
            self._main_stage_hold_steps = 0
            self._reset_integrals()

    def abort(self, reason="MANUAL"):
        self.state, self.timer, self.internal_target_thrust, self.abort_reason = "ABORT", 0.0, 0.0, reason
        self._main_stage_hold_steps = 0
        self._reset_integrals()

    def _reset_integrals(self):
        for pid in (self.pid_thrust, self.pid_orhc, self.pid_frhc,
                    self.pid_tank_ox, self.pid_tank_f, self.pid_mixture):
            pid.integral = 0.0
        self.of_boost_integral = 0.0

    def update(self, target_thrust, current_thrust, telemetry, cav_status):
        (p_bar, rpm_ox, rpm_f, t_cool, of_orhc, of_frhc,
         p_tank_ox, p_tank_f, mdot_ox, mdot_f,
         p_pump_ox, p_pump_f, p_orhc, p_frhc) = telemetry

        if self.state not in ["IDLE", "MECO", "ABORT"]:
            if cav_status[0] or cav_status[1]: self.abort("CAVITATION DETECTED")
            elif p_bar > ABORT_P_MCC_BAR:       self.abort("OVERPRESSURE (MCC)")
            elif rpm_ox > ABORT_RPM_LIMIT or rpm_f > ABORT_RPM_LIMIT: self.abort("TURBINE OVERSPEED")
            elif t_cool > ABORT_T_COOL_K:        self.abort("COOLANT OVERTEMP")
            elif p_tank_ox > ABORT_P_TANK_BAR or p_tank_f > ABORT_P_TANK_BAR: self.abort("TANK OVERPRESSURE")
            self.timer += self.dt

        th_mov = th_mfv = v_orhc = v_frhc = v_auto_ox = v_auto_f = 0.0
        is_ignited = False

        if self.state == "IDLE":
            v_auto_f = self.pid_chilldown.compute(t_cool, self.target_t_chilldown)

        elif self.state == "CHILLDOWN":
            v_auto_f = self.pid_chilldown.compute(t_cool, self.target_t_chilldown)
            # Transizione sensore-based: T_cool sotto soglia + guard timer minimo
            if t_cool <= SEQ_CHILLDOWN_T_COOL_MAX and self.timer >= SEQ_CHILLDOWN_T_MIN:
                self.state, self.timer = "SPIN_PRIME", 0.0
            elif self.timer > SEQ_CHILLDOWN_TO:
                self.abort("CHILLDOWN TIMEOUT")

        elif self.state == "SPIN_PRIME":
            th_mov, th_mfv = 0.40, 0.50  # apertura sufficiente per starter → 7000/12000 RPM
            v_frhc, v_orhc = 0.0, 0.0
            # Autopressurizzazione attiva già in SPIN_PRIME per evitare decadimento P_tank
            v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
            v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)
            # Transizione sensore-based: RPM separati OX/F + serbatoi pressurizzati
            pumps_spinning = rpm_ox >= SEQ_SPINPRIME_RPM_OX_MIN and rpm_f >= SEQ_SPINPRIME_RPM_F_MIN
            tanks_ok       = p_tank_ox >= SEQ_SPINPRIME_P_TANK_MIN and p_tank_f >= SEQ_SPINPRIME_P_TANK_MIN
            if pumps_spinning and tanks_ok and self.timer >= SEQ_SPINPRIME_T_MIN:
                self.state, self.timer = "IGNITION", 0.0
            elif self.timer > SEQ_SPINPRIME_TO:
                self.abort("SPIN_PRIME TIMEOUT")

        elif self.state == "IGNITION":
            is_ignited = True
            th_mov = IGN_TH_MOV
            th_mfv = IGN_TH_MFV
            v_frhc = IGN_V_FRHC
            v_orhc = IGN_V_ORHC
            v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
            v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)
            if rpm_f > SEQ_IGNITION_RPM_F and rpm_ox > SEQ_IGNITION_RPM_OX:
                self.state, self.timer = "RAMP_UP", 0.0
            elif self.timer > SEQ_BOOTSTRAP_TO:
                self.abort("BOOTSTRAP FAILED")

        elif self.state == "RAMP_UP":
            progress = min(1.0, self.timer / SEQ_RAMPUP_DURATION)
            th_mov = RAMP_TH_MOV_0 + (RAMP_TH_MOV_1 - RAMP_TH_MOV_0) * progress
            th_mfv = RAMP_TH_MFV_0 + (RAMP_TH_MFV_1 - RAMP_TH_MFV_0) * progress
            v_frhc = RAMP_V_FRHC_0 + (RAMP_V_FRHC_1 - RAMP_V_FRHC_0) * progress
            v_orhc = RAMP_V_ORHC_0 + (RAMP_V_ORHC_1 - RAMP_V_ORHC_0) * progress
            is_ignited = True
            v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
            v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)
            if (current_thrust > SEQ_RAMPUP_THRUST or progress >= 1.0) and self.timer > 1.0:
                self.state = "MAIN_STAGE"
                # Pre-computa mixture trim dall'OF corrente
                curr_of = mdot_ox / max(mdot_f, 0.01)
                of_error = self.target_of - curr_of
                kp_m = self.pid_mixture.kp
                ki_m = self.pid_mixture.ki
                target_trim = np.clip(kp_m * of_error + 0.5 * of_error, -TRIM_OF_MCC_MAX, TRIM_OF_MCC_MAX)
                if ki_m > 0:
                    self.pid_mixture.integral = target_trim / ki_m
                # Inizializza integrali per continuità PERFETTA con la rampa:
                # Al primo step MAIN_STAGE il rate limiter alzerà internal_target_thrust
                # di rate_step kN → thrust_error = rate_step. L'integrale thrust deve
                # essere tale che kp*error + ki*integral = th_mov - target_trim.
                rate_step = THRUST_RATE_KN_S * self.dt
                ki_t = self.pid_thrust.ki
                kp_t = self.pid_thrust.kp
                th_base_needed = th_mov - target_trim
                if ki_t > 0:
                    # kp*rate_step + ki*integral = th_base_needed
                    self.pid_thrust.integral = (th_base_needed - kp_t * rate_step) / ki_t
                # internal_target_thrust: flag negativo per indicare al primo step
                # MAIN_STAGE di inizializzarsi al thrust reale (il thrust cala
                # fisiologicamente post-transizione)
                self.internal_target_thrust = -1.0
                self.pid_orhc.integral = 0.0
                self.pid_frhc.integral = 0.0
                self.of_boost_integral = 0.0
                # Salva i comandi effettivi del frame di transizione per continuità
                self._transition_th_mov = th_mov
                self._transition_th_mfv = th_mfv
                self._transition_v_orhc = v_orhc
                self._transition_v_frhc = v_frhc
                # Hold comandi per 3 step MAIN_STAGE: il motore deve stabilizzarsi
                self._main_stage_hold_steps = 3
                # Frame di transizione: restituire ESATTAMENTE i comandi RAMP_UP
                v_orhc = RAMP_V_ORHC_1
                v_frhc = RAMP_V_FRHC_1
                v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
                v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)

        elif self.state == "MAIN_STAGE":
            is_ignited = True
            rate = THRUST_RATE_KN_S * self.dt

            if self._main_stage_hold_steps > 0:
                # Hold comandi di transizione: il motore si stabilizza prima che
                # il PID thrust/mixture inizi a reagire (evita oscillazioni)
                self._main_stage_hold_steps -= 1
                # Durante il hold, internal_target evolve con il rate limiter verso
                # target_thrust, così quando il PID parte l'error è contenuto
                self.internal_target_thrust = np.clip(
                    target_thrust,
                    self.internal_target_thrust - rate,
                    self.internal_target_thrust + rate,
                ) if self.internal_target_thrust > 0 else current_thrust
                th_mov = self._transition_th_mov
                th_mfv = self._transition_th_mfv
                v_orhc = self._transition_v_orhc
                v_frhc = self._transition_v_frhc
                v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
                v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)
                if self._main_stage_hold_steps == 0:
                    # Hold appena finito: reinizializza integral per transizione
                    # morbida del PID al prossimo step
                    kp_t = self.pid_thrust.kp
                    ki_t = self.pid_thrust.ki
                    th_base = (self._transition_th_mov + self._transition_th_mfv) / 2.0
                    if ki_t > 0:
                        self.pid_thrust.integral = th_base / ki_t
                    # Reset internal_target al thrust corrente per evitare error spike
                    self.internal_target_thrust = current_thrust
                    # Reinizializza mixture trim
                    curr_of_hold = mdot_ox / max(mdot_f, 0.01)
                    of_error = self.target_of - curr_of_hold
                    kp_m = self.pid_mixture.kp
                    ki_m = self.pid_mixture.ki
                    target_trim = np.clip(kp_m * of_error + 0.5 * of_error, -TRIM_OF_MCC_MAX, TRIM_OF_MCC_MAX)
                    if ki_m > 0:
                        self.pid_mixture.integral = target_trim / ki_m
            else:
                # Post-hold: blended handoff da comandi held a PID output
                if self._main_stage_hold_steps == 0 and not hasattr(self, '_blend_step'):
                    self._main_stage_hold_steps = -1  # negative: blend done, never re-enter
                    self._blend_total = 40
                    self._blend_step = 0
                    self.internal_target_thrust = current_thrust

                if hasattr(self, '_blend_step') and self._blend_step < self._blend_total:
                    self._blend_step += 1

                    # Rate-limit internal_target verso target_thrust
                    self.internal_target_thrust = np.clip(
                        target_thrust,
                        self.internal_target_thrust - rate,
                        self.internal_target_thrust + rate,
                    )

                    curr_of  = mdot_ox / max(mdot_f, 0.01)
                    thrust_error = self.internal_target_thrust - current_thrust
                    ki_t = self.pid_thrust.ki
                    kp_t = self.pid_thrust.kp

                    self.pid_thrust.integral += thrust_error * self.dt
                    max_int_t = 1.0 / ki_t if ki_t > 0 else 0
                    self.pid_thrust.integral = max(-max_int_t, min(max_int_t, self.pid_thrust.integral))

                    th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral
                    of_trim = self.pid_mixture.compute_trim(self.target_of, curr_of, max_trim=TRIM_OF_MCC_MAX)

                    th_base = max(0.0, min(1.0, th_candidate))
                    pid_mov = max(0.0, min(1.0, th_base + of_trim))
                    pid_mfv = max(0.0, min(1.0, th_base - of_trim))

                    alpha = min(1.0, self._blend_step / self._blend_total)
                    th_mov = (1 - alpha) * self._transition_th_mov + alpha * pid_mov
                    th_mfv = (1 - alpha) * self._transition_th_mfv + alpha * pid_mfv

                    if self._blend_step >= self._blend_total:
                        del self._blend_step
                        del self._blend_total

                    v_orhc = np.clip(VALVE_ORHC_NOMINAL - self.pid_orhc.compute_trim(self.target_of_orhc, of_orhc, max_trim=TRIM_ORHC_MAX), 0.0, 1.0)
                    v_frhc = np.clip(VALVE_FRHC_NOMINAL + self.pid_frhc.compute_trim(self.target_of_frhc, of_frhc, max_trim=TRIM_FRHC_MAX), 0.0, 1.0)
                    v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
                    v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)
                else:
                    # Normal: calcola PID da zero
                    curr_of  = mdot_ox / max(mdot_f, 0.01)

                    # Rate limiter verso target_thrust
                    self.internal_target_thrust = np.clip(
                        target_thrust,
                        self.internal_target_thrust - rate,
                        self.internal_target_thrust + rate,
                    )

                    thrust_error = self.internal_target_thrust - current_thrust
                    ki_t = self.pid_thrust.ki
                    kp_t = self.pid_thrust.kp

                    # Calcola output candidato e mixture trim
                    th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral
                    of_trim = self.pid_mixture.compute_trim(self.target_of, curr_of, max_trim=TRIM_OF_MCC_MAX)

                    # Normal: verifica se MOV sarebbe saturo
                    mov_candidate = th_candidate + of_trim
                    if mov_candidate >= 0.85 and thrust_error > 0:
                        # Thrust ceiling più ampio per dare margine al mixture PID
                        self.internal_target_thrust = min(
                            self.internal_target_thrust,
                            current_thrust + 200.0
                        )
                        thrust_error = self.internal_target_thrust - current_thrust
                        th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral
                        if th_candidate + of_trim >= 0.95:
                            self.pid_thrust.integral = (0.95 - of_trim - kp_t * thrust_error) / max(ki_t, 1e-9)
                            th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral
                        if th_candidate + of_trim >= 0.95:
                            self.pid_thrust.integral = (0.95 - of_trim - kp_t * thrust_error) / max(ki_t, 1e-9)
                            th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral
                    else:
                        # Accumula integrale normalmente
                        self.pid_thrust.integral += thrust_error * self.dt
                        max_int = 1.0 / ki_t if ki_t > 0 else 0
                        self.pid_thrust.integral = max(-max_int, min(max_int, self.pid_thrust.integral))
                        th_candidate = kp_t * thrust_error + ki_t * self.pid_thrust.integral

                    th_base = max(0.0, min(1.0, th_candidate))
                    th_mov = max(0.0, min(1.0, th_base + of_trim))
                    th_mfv = max(0.0, min(1.0, th_base - of_trim))

                    # Boost OF quando MOV è saturo e OF è sotto target:
                    # chiudi MFV extra per alzare il rapporto O/F
                    # Boost OF integrale quando MOV è saturo e OF è sotto target:
                    # chiudi MFV extra per alzare il rapporto O/F (correzione limitata per stabilità)
                    if th_mov >= 0.85 and curr_of < self.target_of:
                        of_deficit = self.target_of - curr_of
                        self.of_boost_integral += of_deficit * self.dt
                        self.of_boost_integral = max(0.0, min(0.30, self.of_boost_integral))
                        extra_close = min(0.12, of_deficit * 0.10 + 0.10 * self.of_boost_integral)
                        th_mfv = max(0.0, min(1.0, th_mfv - extra_close))
                    else:
                        # Decadimento integrale se OF è a target o MOV non saturo
                        self.of_boost_integral *= 0.95

                v_orhc = np.clip(VALVE_ORHC_NOMINAL - self.pid_orhc.compute_trim(self.target_of_orhc, of_orhc, max_trim=TRIM_ORHC_MAX), 0.0, 1.0)
                v_frhc = np.clip(VALVE_FRHC_NOMINAL + self.pid_frhc.compute_trim(self.target_of_frhc, of_frhc, max_trim=TRIM_FRHC_MAX), 0.0, 1.0)
                v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
                v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)

        # ── Pressure guard (interlock duro) ───────────────────────────────────
        # Garantisce p_pump_f > p_preburner: condizione necessaria perché CH4
        # possa entrare in entrambi i preburner.
        # Se la pressione di un preburner supera la pompa CH4 → chiude la valvola
        # LOX che alimenta quel preburner (LOX sta gonfiando il preburner troppo).
        if self.state not in ("IDLE", "CHILLDOWN", "SPIN_PRIME", "MECO", "ABORT"):
            # In FFSCC P_preburner/P_pump ≈ 0.93 per design → soglia 0.97 evita falsi trigger.
            # ORHC: LOX entra via th_mov → guard vs pompa LOX (p_pump_ox), non CH4.
            GUARD = 0.97
            if p_orhc > 0.5 and p_pump_ox > 0.5:
                if p_orhc >= p_pump_ox * GUARD:
                    fraction = max(0.0, 1.0 - (p_orhc - p_pump_ox * GUARD) /
                                   max(p_pump_ox * 0.05, 0.1))
                    th_mov = th_mov * fraction
            # FRHC: LOX entra via v_frhc (cross-bleed) → guard vs pompa CH4 (p_pump_f).
            if p_frhc > 0.5 and p_pump_f > 0.5:
                if p_frhc >= p_pump_f * GUARD:
                    fraction = max(0.0, 1.0 - (p_frhc - p_pump_f * GUARD) /
                                   max(p_pump_f * 0.05, 0.1))
                    v_frhc = v_frhc * fraction

        return th_mov, th_mfv, v_orhc, v_frhc, v_auto_ox, v_auto_f, is_ignited, self.state


class ProportionalValve:
    def __init__(self, tau):
        self.tau = tau

    def get_derivative(self, cmd_pos, actual_pos):
        return (cmd_pos - actual_pos) / self.tau
