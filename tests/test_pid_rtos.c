#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "freertos_sim.h"
#include "motor_pid.h"

/* External simulation runner from src/main.c */
extern StepResponseMetrics_t Run_RTOS_Simulation(
    const char* profile_name,
    double kp,
    double ki,
    double kd,
    bool anti_windup,
    bool deriv_on_meas,
    const char* csv_out_path,
    const char* schedule_csv_path,
    TaskControlBlock_t task_stats_out[3]
);

int main(void) {
    printf("Running Embedded C PID + FreeRTOS Verification Test Suite...\n");
    int passed = 0;
    int total  = 6;

    /* Test 1: FreeRTOS Queue FIFO Integrity & Overrun Protection */
    QueueHandle_t q = xQueueCreate(4, sizeof(SensorMsg_t));
    assert(q != NULL);
    for (int i = 0; i < 4; i++) {
        SensorMsg_t m = { .timestamp_ms = (uint32_t)i, .filtered_rpm = 100.0 * i };
        assert(xQueueSend(q, &m, 0) == pdPASS);
    }
    SensorMsg_t extra = { .timestamp_ms = 99 };
    assert(xQueueSend(q, &extra, 0) == pdFAIL); /* Queue full -> must reject cleanly */
    assert(q->total_overruns == 1);
    SensorMsg_t rx;
    assert(xQueueReceive(q, &rx, 0) == pdPASS && rx.timestamp_ms == 0);
    free(q);
    printf("  [PASS] TC-01: FreeRTOS Queue FIFO order and capacity overrun protection\n");
    passed++;

    /* Test 2: FreeRTOS Mutex Semaphore Lock/Unlock & Contention Tracking */
    SemaphoreHandle_t mtx = xSemaphoreCreateMutex();
    assert(mtx != NULL);
    assert(xSemaphoreTake(mtx, "TaskA") == pdPASS);
    assert(xSemaphoreTake(mtx, "TaskB") == pdFAIL); /* Locked by TaskA */
    assert(mtx->contention_count == 1);
    assert(xSemaphoreGive(mtx) == pdPASS);
    assert(xSemaphoreTake(mtx, "TaskB") == pdPASS);
    assert(xSemaphoreGive(mtx) == pdPASS);
    free(mtx);
    printf("  [PASS] TC-02: FreeRTOS Mutex mutual exclusion and contention counter\n");
    passed++;

    /* Test 3: Anti-Windup Clamping Verification under Actuator Saturation */
    PIDController_t pid_aw, pid_no_aw;
    PID_Init(&pid_aw, 0.05, 1.5, 0.0, 0.0, 24.0, true, true);
    PID_Init(&pid_no_aw, 0.05, 1.5, 0.0, 0.0, 24.0, false, false);
    for (int k = 0; k < 100; k++) {
        PID_Compute(&pid_aw, 1500.0, 0.0, 0.002);
        PID_Compute(&pid_no_aw, 1500.0, 0.0, 0.002);
    }
    assert(pid_aw.output_sat == 24.0);
    assert(pid_aw.integral_sum < pid_no_aw.integral_sum);
    printf("  [PASS] TC-03: PID Anti-Windup clamps integrator accumulation during 24V saturation\n");
    passed++;

    /* Test 4: Before vs. After Tuning Overshoot & Settling Time Improvement */
    TaskControlBlock_t stats[3];
    StepResponseMetrics_t before_m = Run_RTOS_Simulation("Before", 0.045, 1.850, 0.0, false, false, NULL, NULL, NULL);
    StepResponseMetrics_t after_m  = Run_RTOS_Simulation("After",  0.058, 0.620, 0.00085, true, true, NULL, NULL, stats);

    assert(before_m.overshoot_pct > 15.0); /* Untuned exhibits significant windup overshoot */
    assert(after_m.overshoot_pct < 2.5);   /* Tuned PID keeps overshoot < 2.5% */
    assert(after_m.settling_time_2pct_ms < before_m.settling_time_2pct_ms * 0.55);
    printf("  [PASS] TC-04: Tuned PID reduces overshoot from %.2f%% to %.2f%% and settling time from %.1fms to %.1fms\n",
           before_m.overshoot_pct, after_m.overshoot_pct,
           before_m.settling_time_2pct_ms, after_m.settling_time_2pct_ms);
    passed++;

    /* Test 5: External Load Torque Disturbance Rejection (+0.18 N*m at t = 1.0s) */
    assert(after_m.disturbance_drop_rpm < 110.0);
    assert(after_m.disturbance_recovery_ms < 220.0);
    assert(after_m.steady_state_err_rpm < 2.0);
    printf("  [PASS] TC-05: Load disturbance (+0.18 N*m) rejected with max dip %.1f RPM and recovery in %.1f ms\n",
           after_m.disturbance_drop_rpm, after_m.disturbance_recovery_ms);
    passed++;

    /* Test 6: Rate-Monotonic RTOS Task Schedulability & Zero Deadline Misses */
    assert(stats[0].deadline_misses == 0);
    assert(stats[1].deadline_misses == 0);
    assert(stats[2].deadline_misses == 0);
    assert(stats[0].max_jitter_us == 0.0); /* Highest priority task has 0 preemption jitter */
    printf("  [PASS] TC-06: All 3 RTOS tasks met 100%% of hard deadlines (0 misses; Prio-3 jitter = 0.0 us)\n");
    passed++;

    printf("\nAll %d/%d Embedded C Verification Tests PASSED.\n", passed, total);
    return 0;
}
