#include "freertos_sim.h"
#include <stdlib.h>
#include <string.h>

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize) {
    if (uxQueueLength == 0 || uxQueueLength > RTOS_QUEUE_MAX_CAPACITY || uxItemSize > 64) {
        return NULL;
    }
    QueueControlBlock_t* q = (QueueControlBlock_t*)calloc(1, sizeof(QueueControlBlock_t));
    if (!q) return NULL;
    q->item_size = uxItemSize;
    q->capacity  = uxQueueLength;
    q->head      = 0;
    q->tail      = 0;
    q->count     = 0;
    return q;
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void* pvItemToQueue, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    if (!xQueue || !pvItemToQueue) return pdFAIL;
    if (xQueue->count >= xQueue->capacity) {
        xQueue->total_overruns++;
        return pdFAIL;
    }
    uint8_t* dest = xQueue->storage + (xQueue->tail * xQueue->item_size);
    memcpy(dest, pvItemToQueue, xQueue->item_size);
    xQueue->tail = (xQueue->tail + 1) % xQueue->capacity;
    xQueue->count++;
    xQueue->total_sent++;
    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void* pvBuffer, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    if (!xQueue || !pvBuffer || xQueue->count == 0) return pdFAIL;
    const uint8_t* src = xQueue->storage + (xQueue->head * xQueue->item_size);
    memcpy(pvBuffer, src, xQueue->item_size);
    xQueue->head = (xQueue->head + 1) % xQueue->capacity;
    xQueue->count--;
    xQueue->total_received++;
    return pdPASS;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    MutexControlBlock_t* m = (MutexControlBlock_t*)calloc(1, sizeof(MutexControlBlock_t));
    if (!m) return NULL;
    m->locked = false;
    m->holder_task = NULL;
    return m;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xMutex, const char* task_name) {
    if (!xMutex) return pdFAIL;
    if (xMutex->locked) {
        xMutex->contention_count++;
        return pdFAIL;
    }
    xMutex->locked = true;
    xMutex->holder_task = task_name;
    xMutex->lock_count++;
    return pdPASS;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xMutex) {
    if (!xMutex || !xMutex->locked) return pdFAIL;
    xMutex->locked = false;
    xMutex->holder_task = NULL;
    return pdPASS;
}
