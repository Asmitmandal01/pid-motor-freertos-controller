#include "motor_pid.h"

void PID_Init(
    PIDController_t* pid,
    double kp,
    double ki,
    double kd,
    double out_min,
    double out_max,
    bool anti_windup,
    bool deriv_on_meas
) {
    pid->kp                   = kp;
    pid->ki                   = ki;
    pid->kd                   = kd;
    pid->kb_aw                = 25.0;  /* Back-calculation tracking rate */
    pid->deriv_lpf_alpha      = 0.25;  /* Derivative low-pass filter smoothing */
    pid->out_min              = out_min;
    pid->out_max              = out_max;
    pid->enable_anti_windup   = anti_windup;
    pid->deriv_on_measurement = deriv_on_meas;
    PID_Reset(pid);
}

void PID_Reset(PIDController_t* pid) {
    pid->integral_sum     = 0.0;
    pid->prev_error       = 0.0;
    pid->prev_measurement = 0.0;
    pid->filtered_deriv   = 0.0;
    pid->p_term           = 0.0;
    pid->i_term           = 0.0;
    pid->d_term           = 0.0;
    pid->output_unsat     = 0.0;
    pid->output_sat       = 0.0;
}

double PID_Compute(PIDController_t* pid, double setpoint_rpm, double measured_rpm, double dt_s) {
    if (dt_s <= 0.0) return pid->output_sat;

    double error = setpoint_rpm - measured_rpm;

    /* 1. Proportional Term */
    pid->p_term = pid->kp * error;

    /* 2. Derivative Term (Derivative-on-Measurement avoids setpoint step kick) */
    double raw_deriv = 0.0;
    if (pid->deriv_on_measurement) {
        raw_deriv = -(measured_rpm - pid->prev_measurement) / dt_s;
    } else {
        raw_deriv = (error - pid->prev_error) / dt_s;
    }
    pid->filtered_deriv = pid->deriv_lpf_alpha * raw_deriv + (1.0 - pid->deriv_lpf_alpha) * pid->filtered_deriv;
    pid->d_term = pid->kd * pid->filtered_deriv;

    /* 3. Integral Term with Optional Anti-Windup Clamping & Back-Calculation */
    double candidate_integral = pid->integral_sum + error * dt_s;
    double candidate_i_term   = pid->ki * candidate_integral;
    double pre_sat            = pid->p_term + candidate_i_term + pid->d_term;

    double sat = pre_sat;
    if (sat > pid->out_max) sat = pid->out_max;
    if (sat < pid->out_min) sat = pid->out_min;

    if (pid->enable_anti_windup) {
        /* Conditional integration clamping: stop integrating if actuator is saturated in same direction */
        bool saturating_high = (pre_sat > pid->out_max) && (error > 0.0);
        bool saturating_low  = (pre_sat < pid->out_min) && (error < 0.0);
        if (!saturating_high && !saturating_low) {
            pid->integral_sum = candidate_integral;
        }
    } else {
        /* Unclamped integration (demonstrates classic integrator windup overshoot) */
        pid->integral_sum = candidate_integral;
    }

    pid->i_term       = pid->ki * pid->integral_sum;
    pid->output_unsat = pid->p_term + pid->i_term + pid->d_term;

    pid->output_sat = pid->output_unsat;
    if (pid->output_sat > pid->out_max) pid->output_sat = pid->out_max;
    if (pid->output_sat < pid->out_min) pid->output_sat = pid->out_min;

    pid->prev_error       = error;
    pid->prev_measurement = measured_rpm;

    return pid->output_sat;
}
