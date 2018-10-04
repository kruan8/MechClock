/*
 * timer.c
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#include "timer.h"

static volatile uint32_t g_nTicks = 0;

static PtrSysTickCallback pSysTickCallback = 0;


void Timer_Init()
{
  SystemCoreClockUpdate();

  // set Systick to 1ms
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
}

void Timer_Delay_ms(uint32_t delay_ms)
{
  uint32_t nEndTime = g_nTicks + delay_ms;
  while (g_nTicks < nEndTime);
}

uint32_t Timer_GetTicks_ms()
{
  return g_nTicks;
}

void Timer_SetSysTickCallback(PtrSysTickCallback pFunction)
{
  pSysTickCallback = pFunction;
}

void SysTick_Handler(void)
{
  g_nTicks++;

  if (pSysTickCallback)
  {
    pSysTickCallback();
  }
}
