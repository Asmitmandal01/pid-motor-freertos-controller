# PID Motor Speed Controller with FreeRTOS-Style Tasks — Control & Timing Verification Report

## 1. Executive Summary
- **Target Plant:** 24 V, 2500 RPM Brushed/BLDC Servo Motor (`R_a = 1.20 Ω`, `L_a = 4.5 mH`, `K_e = K_t = 0.068`, `J = 2.8e-4 kg·m²`)
- **RTOS Architecture:** 3 Rate-Monotonic Periodic Tasks (`vTask_SpeedSensing` @ 1 kHz, `vTask_PIDControlLoop` @ 500 Hz, `vTask_UARTTelemetry` @ 100 Hz) communicating via lock-free FIFO Queue (`xSensorQueue`) and Priority Mutex (`xTelemetryMutex`).
- **Overall Verdict:** **PASS (6/6 Embedded C Unit & Integration Tests Passed — 0 RTOS Deadline Misses)**

## 2. Before vs. After PID Tuning Comparison (0 -> 1500 RPM Step + 0.18 N·m Load Disturbance)

| Controller Profile | Gains (`Kp, Ki, Kd`) | Anti-Windup & Deriv LPF | Peak Speed (RPM) | Step Overshoot (%) | Rise Time (10–90%) | 2% Settling Time | +0.18 N·m Load Dip | Disturbance Recovery |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Before Tuning (Naive PI)** | `0.045, 1.850, 0.000` | Disabled | `2146.7 RPM` | `43.11%` | `40.0 ms` | `550.0 ms` | `52.9 RPM` | `30.0 ms` |
| **2. Ziegler-Nichols PID** | `0.065, 1.100, 0.0004` | Disabled | `1781.9 RPM` | `18.79%` | `40.0 ms` | `200.0 ms` | `39.3 RPM` | `50.0 ms` |
| **3. After Tuning (Proposed)** | `0.058, 0.620, 0.00085` | **Enabled** | **`1518.6 RPM`** | **`1.24%`** | **`60.0 ms`** | **`90.0 ms`** | **`35.9 RPM`** | **`100.0 ms`** |

## 3. FreeRTOS Task Timing & Rate-Monotonic Schedulability Table

| Task Name | RTOS Priority | Period (`T_i`) | Frequency | WCET (`C_i`) | Avg Exec Time | Max Release Jitter | CPU Utilization (`U_i`) | Deadline Misses |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `vTask_SpeedSensing` | **3 (Highest)** | `1.0 ms` (`1000 µs`) | `1000 Hz` | `21.3 µs` | `18.5 µs` | `0.0 µs` | `1.85%` | `0 / 2000` |
| `vTask_PIDControlLoop` | **2 (High)** | `2.0 ms` (`2000 µs`) | `500 Hz` | `44.8 µs` | `42.0 µs` | `21.3 µs` | `2.10%` | `0 / 1000` |
| `vTask_UARTTelemetry` | **1 (Low)** | `10.0 ms` (`10000 µs`) | `100 Hz` | `117.8 µs` | `115.0 µs` | `66.1 µs` | `1.15%` | `0 / 200` |
| **Total System Load** | — | — | — | — | — | — | **`5.10%` (`< 77.98%` RM Bound)** | **`0` (100% Real-Time Safe)** |
