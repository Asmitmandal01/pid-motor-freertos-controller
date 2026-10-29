#ifndef FREERTOS_SIM_H
#define FREERTOS_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* FreeRTOS-compatible type definitions for portable Embedded C */
typedef uint32_t TickType_t;
typedef int32_t  BaseType_t;
typedef uint32_t UBaseType_t;

#define pdTRUE   ((BaseType_t) 1)
#define pdFALSE  ((BaseType_t) 0)
#define pdPASS   pdTRUE
#define pdFAIL   pdFALSE

/* Ring-buffer Queue Handle (models FreeRTOS QueueHandle_t) */
#define RTOS_QUEUE_MAX_CAPACITY 32

typedef struct {
    uint8_t  storage[RTOS_QUEUE_MAX_CAPACITY * 64];
    size_t   item_size;
    size_t   capacity;
    size_t   head;
    size_t   tail;
    size_t   count;
    uint32_t total_sent;
    uint32_t total_received;
    uint32_t total_overruns;
} QueueControlBlock_t;

typedef QueueControlBlock_t* QueueHandle_t;

/* Mutex Semaphore Handle (models FreeRTOS SemaphoreHandle_t) */
typedef struct {
    bool        locked;
    const char* holder_task;
    uint32_t    lock_count;
    uint32_t    contention_count;
} MutexControlBlock_t;

typedef MutexControlBlock_t* SemaphoreHandle_t;

/* Periodic RTOS Task Function Prototype */
typedef void (*TaskFunction_t)(void* pvParameters);

/* Task Control Block (TCB) with Real-Time Execution Profiling */
typedef struct {
    const char*    task_name;
    UBaseType_t    priority;         /* Higher number = Higher priority (FreeRTOS convention) */
    TickType_t     period_ticks;     /* Task activation period in ticks (1 tick = 100 us = 0.1 ms) */
    TickType_t     next_release_tick;
    TaskFunction_t task_fn;
    void*          task_params;

    /* Real-time timing metrics (in microseconds) */
    uint32_t       invocations;
    double         wcet_us;          /* Worst-Case Execution Time (us) */
    double         bcet_us;          /* Best-Case Execution Time (us) */
    double         total_exec_us;    /* Cumulative execution time (us) */
    double         max_jitter_us;    /* Max release jitter (us) */
    uint32_t       deadline_misses;  /* Count of executions exceeding period */
} TaskControlBlock_t;

/* FreeRTOS-style Queue & Semaphore API */
QueueHandle_t     xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
BaseType_t        xQueueSend(QueueHandle_t xQueue, const void* pvItemToQueue, TickType_t xTicksToWait);
BaseType_t        xQueueReceive(QueueHandle_t xQueue, void* pvBuffer, TickType_t xTicksToWait);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t        xSemaphoreTake(SemaphoreHandle_t xMutex, const char* task_name);
BaseType_t        xSemaphoreGive(SemaphoreHandle_t xMutex);

#endif /* FREERTOS_SIM_H */
