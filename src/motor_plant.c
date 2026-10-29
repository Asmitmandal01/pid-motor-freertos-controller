#include "motor_pid.h"
#include <math.h>

#define RADS_TO_RPM (60.0 / (2.0 * 3.14159265358979323846))

void DCMotor_Init(DCMotorPlant_t* motor) {
    /* Industrial 24V, 2500 RPM Brushed/BLDC Servo Motor Parameters */
    motor->R_ohm          = 1.20;     /* 1.2 Ohm armature resistance */
    motor->L_henry        = 0.0045;   /* 4.5 mH armature inductance (tau_elec = 3.75 ms) */
    motor->Ke_v_per_rads  = 0.068;    /* Back-EMF constant: ~17.8 V at 2500 RPM */
    motor->Kt_nm_per_a    = 0.068;    /* Torque constant: 0.068 N*m/A */
    motor->J_kgm2         = 0.00028;  /* Rotor + inertia load (tau_mech ~ 45 ms) */
    motor->B_nms          = 0.00035;  /* Viscous damping */
    motor->v_supply_max   = 24.0;

    motor->current_a      = 0.0;
    motor->omega_rads     = 0.0;
    motor->speed_rpm      = 0.0;
    motor->load_torque_nm = 0.0;
    motor->applied_volts  = 0.0;
}

void DCMotor_Step(DCMotorPlant_t* motor, double dt_s, double pwm_voltage_v, double load_torque_nm) {
    /* Clamp voltage to physical H-bridge supply rails [0V, 24V] */
    if (pwm_voltage_v > motor->v_supply_max) pwm_voltage_v = motor->v_supply_max;
    if (pwm_voltage_v < 0.0)                 pwm_voltage_v = 0.0;

    motor->applied_volts  = pwm_voltage_v;
    motor->load_torque_nm = load_torque_nm;

    /* Sub-step integration (10 sub-steps per call) for stiff electrical inductance accuracy */
    int substeps = 10;
    double h = dt_s / (double)substeps;

    for (int k = 0; k < substeps; k++) {
        double back_emf_v = motor->Ke_v_per_rads * motor->omega_rads;
        double di_dt = (pwm_voltage_v - motor->R_ohm * motor->current_a - back_emf_v) / motor->L_henry;
        motor->current_a += di_dt * h;
        if (motor->current_a < 0.0) motor->current_a = 0.0; /* Unidirectional quadrant 1 drive */

        double elec_torque_nm = motor->Kt_nm_per_a * motor->current_a;
        double net_torque_nm  = elec_torque_nm - (motor->B_nms * motor->omega_rads) - load_torque_nm;
        double domega_dt      = net_torque_nm / motor->J_kgm2;
        motor->omega_rads += domega_dt * h;
        if (motor->omega_rads < 0.0) motor->omega_rads = 0.0;
    }

    motor->speed_rpm = motor->omega_rads * RADS_TO_RPM;
}
