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
    SEQ_SPINPRIME_RPM_MIN, SEQ_SPINPRIME_P_TANK_MIN, SEQ_SPINPRIME_T_MIN, SEQ_SPINPRIME_TO,
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

    def start_sequence(self):
        if self.state in ["IDLE", "MECO", "ABORT"]:
            self.state, self.timer, self.abort_reason = "CHILLDOWN", 0.0, ""

    def meco(self):
        if self.state not in ["IDLE", "MECO", "ABORT"]:
            self.state, self.timer, self.internal_target_thrust, self.abort_reason = "MECO", 0.0, 0.0, ""
            self._reset_integrals()

    def abort(self, reason="MANUAL"):
        self.state, self.timer, self.internal_target_thrust, self.abort_reason = "ABORT", 0.0, 0.0, reason
        self._reset_integrals()

    def _reset_integrals(self):
        for pid in (self.pid_thrust, self.pid_orhc, self.pid_frhc,
                    self.pid_tank_ox, self.pid_tank_f, self.pid_mixture):
            pid.integral = 0.0

    def update(self, target_thrust, current_thrust, telemetry, cav_status):
        (p_bar, rpm_ox, rpm_f, t_cool, of_orhc, of_frhc,
         p_tank_ox, p_tank_f, mdot_ox, mdot_f) = telemetry

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
            th_mov, th_mfv = 0.10, 0.20
            v_frhc, v_orhc = 0.0, 0.0
            # Transizione sensore-based: RPM confermano flusso + serbatoi pressurizzati
            pumps_spinning = rpm_ox >= SEQ_SPINPRIME_RPM_MIN and rpm_f >= SEQ_SPINPRIME_RPM_MIN
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
                self.state, self.internal_target_thrust = "MAIN_STAGE", current_thrust
                th_base = (th_mov + th_mfv) / 2.0
                if self.pid_thrust.ki > 0:
                    self.pid_thrust.integral = th_base / self.pid_thrust.ki
                self.pid_mixture.integral = 0.0
                self.pid_orhc.integral = 0.0
                self.pid_frhc.integral = 0.0

        elif self.state == "MAIN_STAGE":
            is_ignited = True
            rate = THRUST_RATE_KN_S * self.dt
            self.internal_target_thrust = np.clip(
                target_thrust,
                self.internal_target_thrust - rate,
                self.internal_target_thrust + rate,
            )
            th_base  = self.pid_thrust.compute(self.internal_target_thrust, current_thrust)
            curr_of  = mdot_ox / max(mdot_f, 0.01)
            of_trim  = self.pid_mixture.compute_trim(self.target_of, curr_of, max_trim=TRIM_OF_MCC_MAX)
            th_mov   = max(0.0, min(1.0, th_base + of_trim))
            th_mfv   = max(0.0, min(1.0, th_base - of_trim))
            v_orhc = np.clip(VALVE_ORHC_NOMINAL - self.pid_orhc.compute_trim(self.target_of_orhc, of_orhc, max_trim=TRIM_ORHC_MAX), 0.0, 1.0)
            v_frhc = np.clip(VALVE_FRHC_NOMINAL + self.pid_frhc.compute_trim(self.target_of_frhc, of_frhc, max_trim=TRIM_FRHC_MAX), 0.0, 1.0)
            v_auto_ox = self.pid_tank_ox.compute(self.target_p_tank, p_tank_ox)
            v_auto_f  = self.pid_tank_f.compute(self.target_p_tank, p_tank_f)

        return th_mov, th_mfv, v_orhc, v_frhc, v_auto_ox, v_auto_f, is_ignited, self.state


class ProportionalValve:
    def __init__(self, tau):
        self.tau = tau

    def get_derivative(self, cmd_pos, actual_pos):
        return (cmd_pos - actual_pos) / self.tau
