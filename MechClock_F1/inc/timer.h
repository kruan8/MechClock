/*
 * timer.h
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stm32f10x.h>

typedef void(*PtrSysTickCallback) (void);

void Timer_Init();
void Timer_Delay_ms(uint32_t delay_ms);
uint32_t Timer_GetTicks_ms();
void Timer_SetSysTickCallback(PtrSysTickCallback pFunction);

#endif /* TIMER_H_ */
