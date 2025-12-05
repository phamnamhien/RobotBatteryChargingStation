/*
 * hsm_timer.c
 *
 *  Created on: Feb 5, 2025
 *      Author: phamn
 *
 *  hsmTimer Library: Provides functions to create, start, stop, and delete software timers
 *  using Timer2 with a 1ms tick.
 */

#include "hsm_timer.h"

/* Maximum number of timers */
#define TIMER_MAX        MAX_TIMERS

/* Runtime structure for managing a timer */
typedef struct {
    uint8_t active;       /* 1: running, 0: stopped */
    uint32_t reload;      /* Reload time in ms */
    uint32_t counter;     /* Remaining counter in ms */
} hsmTimerRuntime_t;

/* Static pool of timers */
static hsmTimer_t timerPool[TIMER_MAX];
/* Array to indicate if a timer slot is allocated */
static uint8_t timerAllocated[TIMER_MAX] = {0};
/* Array to hold the runtime data for each timer */
static hsmTimerRuntime_t timerRuntime[TIMER_MAX] = {0};

/**
 * @brief  Find the index of a timer in the pool based on hsmTimerId.
 * @param  timer_id: pointer to the timer to search for.
 * @retval Returns the index if found, otherwise returns -1.
 */
static int8_t timer_get_index(hsmTimerId timer_id) {
    int8_t i;
    for (i = 0; i < TIMER_MAX; i++) {
        if (timerAllocated[i] && (timer_id == &timerPool[i])) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief  Create a new software timer.
 * @param  timer_def: pointer to the timer definition structure (contains the callback).
 * @param  type: timer type (one-shot or periodic).
 * @param  argument: parameter to pass to the callback.
 * @retval Returns a pointer to the created timer, or NULL if no slot is available.
 */
hsmTimerId hsmTimerCreate(const hsmTimerDef_t *timer_def, hsmTimerType type, void *argument) {
    uint8_t i;
    for (i = 0; i < TIMER_MAX; i++) {
        if (timerAllocated[i] == 0) {
            /* Mark this slot as allocated */
            timerAllocated[i] = 1;
            /* Assign the callback and argument */
            timerPool[i].callback = timer_def->callback;
            timerPool[i].argument = argument;
            timerPool[i].type = type;
            /* Assign the timer handle (using Timer2) */
            timerPool[i].htim = hsm_htim;
            /* Initialize runtime data */
            timerRuntime[i].active = 0;
            timerRuntime[i].reload = 0;
            timerRuntime[i].counter = 0;
            return &timerPool[i];
        }
    }
    return NULL;
}

/**
 * @brief  Start a software timer.
 * @param  timer_id: pointer to the timer to start.
 * @param  millisec: duration of the timer in milliseconds.
 * @retval Returns hsmOK if successful, otherwise returns an error.
 */
hsmStatus hsmTimerStart(hsmTimerId timer_id, uint32_t millisec) {
    int8_t idx = timer_get_index(timer_id);
    if (idx < 0 || millisec == 0) {
        return hsmErrorParameter;
    }

    /* Set the timer duration */
    timerRuntime[idx].reload = millisec;
    timerRuntime[idx].counter = millisec;
    timerRuntime[idx].active = 1;

    return hsmOK;
}

/**
 * @brief  Stop a software timer.
 * @param  timer_id: pointer to the timer to stop.
 * @retval Returns hsmOK if successful, otherwise returns an error.
 */
hsmStatus hsmTimerStop(hsmTimerId timer_id) {
    int8_t idx = timer_get_index(timer_id);
    if (idx < 0) {
        return hsmErrorParameter;
    }

    timerRuntime[idx].active = 0;
    return hsmOK;
}

/**
 * @brief  Delete a software timer.
 * @param  timer_id: pointer to the timer to delete.
 * @retval Returns hsmOK if successful, otherwise returns an error.
 */
hsmStatus hsmTimerDelete(hsmTimerId timer_id) {
    int8_t idx = timer_get_index(timer_id);
    if (idx < 0) {
        return hsmErrorParameter;
    }

    /* Stop the timer and mark the slot as free */
    timerRuntime[idx].active = 0;
    timerAllocated[idx] = 0;
    /* Clear the callback and argument */
    timerPool[idx].callback = NULL;
    timerPool[idx].argument = NULL;

    return hsmOK;
}

/**
 * @brief  Timer2 interrupt handler (called every 1ms).
 *         This function should be called from Timer2's ISR.
 */
void HSM_TIM_IRQHandler(void) {
    uint8_t i;

    /* Process tick for each software timer */
    for (i = 0; i < TIMER_MAX; i++) {
        if (timerAllocated[i] && timerRuntime[i].active) {
            if (timerRuntime[i].counter > 0) {
                timerRuntime[i].counter--;
            }
            /* When the counter reaches zero, invoke the callback */
            if (timerRuntime[i].counter == 0) {
                if (timerPool[i].callback != NULL) {
                    timerPool[i].callback(timerPool[i].argument);
                }
                /* For one-shot timers, stop the timer.
                   For periodic timers, reload the counter. */
                if (timerPool[i].type == hsmTimerOnce) {
                    timerRuntime[i].active = 0;
                } else {
                    timerRuntime[i].counter = timerRuntime[i].reload;
                }
            }
        }
    }

    /* Clear the interrupt flag and call the HAL handler if needed (depending on HAL) */
    HAL_TIM_IRQHandler(&hsm_htim);
}

/**
 * @brief  Initialize Timer2 to generate an interrupt every 1ms.
 *         Call this function in main() before using the software timers.
 */
void hsmTimerInit(void) {
    /* Start Timer2 in interrupt mode with a 1ms period.
       It is assumed that Timer2 is properly configured in tim.c.
       If not, you need to configure Timer2 in Base mode.
     */
    HAL_TIM_Base_Start_IT(&hsm_htim);
}
