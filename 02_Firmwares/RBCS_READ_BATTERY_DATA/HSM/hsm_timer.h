/*
 * hsm_timer.h
 *
 *  Created on: Feb 5, 2025
 *      Author: phamn
 */

#ifndef HSM_TIMER_H_
#define HSM_TIMER_H_

#include "main.h"
#include "tim.h"


#define MAX_TIMERS	16
#define hsm_htim	htim2
#define HSM_TIM		TIM2


typedef void (*hsmTimerCallback_t)(void *argument);

typedef enum {
    hsmOK = 0,
    hsmErrorParameter,
    hsmErrorMemory
} hsmStatus;

typedef enum {
    hsmTimerOnce = 0,
    hsmTimerPeriodic
} hsmTimerType;

typedef struct {
    TIM_HandleTypeDef htim;
    hsmTimerCallback_t callback;
    void *argument;
    hsmTimerType type;
} hsmTimer_t;

typedef hsmTimer_t* hsmTimerId;

typedef struct {
    const char *name;
    hsmTimerCallback_t callback;
} hsmTimerDef_t;

hsmTimerId hsmTimerCreate(const hsmTimerDef_t *timer_def, hsmTimerType type, void *argument);
hsmStatus hsmTimerStart(hsmTimerId timer_id, uint32_t millisec);
hsmStatus hsmTimerStop(hsmTimerId timer_id);
hsmStatus hsmTimerDelete(hsmTimerId timer_id);

void hsmTimerInit(void);
void HSM_TIM_IRQHandler(void);

#endif /* HSM_TIMER_H_ */
