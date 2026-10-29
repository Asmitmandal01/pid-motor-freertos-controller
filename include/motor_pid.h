#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos_sim.h"

/* =========================================================================
 * 1. Electro-Mechanical Brushed/BLDC Motor Plant Model
 *    - Armature circuit: V_a = R_a * i_a + L_a * (di_a/dt) + K_e * omega
 *    - Mechanical shaft: J * (domega/dt) = K_t * i_a - B * omega - T_load
 * ========================================================================= */
typedef struct {
    /* Physical constants */
    double R_ohm;           /* Armature resistance (Ohms) */
    double L_henry;         /* Armature inductance (Henrys) */
    double Ke_v_per_rads;   /* Back-EMF constant (V / (rad/s)) */
    double Kt_nm_per_a;     /* Torque constant (N*m / A) */
    double J_kgm2;          /* Rotor + load moment of inertia (kg*m^2) */
    double B_nms;           /* Viscous friction coefficient (N*m*s/rad) */
    double v_supply_max;    /* H-Bridge supply rail voltage (24.0 V) */

    /* Dynamic states */
    double current_a;       /* Armature current i_a (Amps) */
    double omega_rads;      /* Shaft angular velocity (rad/s) */
    double speed_rpm;       /* Shaft speed in RPM */
    double load_torque_nm;  /* External disturbance torque T_load (N*m) */
    double applied_volts;   /* Applied PWM average voltage (V) */
} DCMotorPlant_t;

void DCMotor_Init(DCMotorPlant_t* motor);
void DCMotor_Step(DCMotorPlant_t* motor, double dt_s, double pwm_voltage_v, double load_torque_nm);

/* =========================================================================
 * 2. Discrete-Time PID Controller with Anti-Windup & Derivative Filter
 *    - Supports both Uncalibrated/Naive PID (for Before-tuning baseline)
 *      and Tuned Industrial PID (Clamped/Back-Calculation Anti-Windup +
 *      Derivative-on-Measurement to eliminate setpoint-step kick)
 * ========================================================================= */
typedef struct {
    double kp;                  /* Proportional gain */
    double ki;                  /* Integral gain */
    double kd;                  /* Derivative gain */
    double kb_aw;               /* Anti-windup back-calculation gain */
    double deriv_lpf_alpha;     /* Low-pass filter coefficient on derivative term */
    double out_min;             /* Minimum actuator output (0.0 V) */
    double out_max;             /* Maximum actuator output (24.0 V) */
    bool   enable_anti_windup;  /* Enable integrator clamping / back-calculation */
    bool   deriv_on_measurement;/* True = -Kd * d(meas)/dt (prevents derivative kick) */

    /* Internal controller states */
    double integral_sum;
    double prev_error;
    double prev_measurement;
    double filtered_deriv;
    double p_term;
    double i_term;
    double d_term;
    double output_unsat;
    double output_sat;
} PIDController_t;

void   PID_Init(PIDController_t* pid, double kp, double ki, double kd, double out_min, double out_max, bool anti_windup, bool deriv_on_meas);
void   PID_Reset(PIDController_t* pid);
double PID_Compute(PIDController_t* pid, double setpoint_rpm, double measured_rpm, double dt_s);

/* =========================================================================
 * 3. Inter-Task Communication Payloads (Queue & Mutex-Protected Telemetry)
 * ========================================================================= */
typedef struct {
    uint32_t timestamp_ms;
    double   raw_encoder_rpm;
    double   filtered_rpm;
    double   armature_current_a;
} SensorMsg_t;

typedef struct {
    uint32_t timestamp_ms;
    double   setpoint_rpm;
    double   measured_rpm;
    double   error_rpm;
    double   pwm_voltage_v;
    double   pwm_duty_pct;
    double   armature_current_a;
    double   load_torque_nm;
    double   p_term;
    double   i_term;
    double   d_term;
} ControlTelemetry_t;

/* Step-Response Performance Metrics */
typedef struct {
    const char* profile_name;
    double      kp;
    double      ki;
    double      kd;
    double      target_rpm;
    double      peak_rpm;
    double      overshoot_pct;
    double      rise_time_ms;          /* 10% to 90% of target */
    double      settling_time_2pct_ms; /* Time to enter and stay within +/- 2% band */
    double      steady_state_err_rpm;
    double      disturbance_drop_rpm;  /* Max RPM dip under +0.15 N*m load step at t=1.0s */
    double      disturbance_recovery_ms;
} StepResponseMetrics_t;

#endif /* MOTOR_PID_H */
