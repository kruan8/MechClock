/*
 * app.c
 *
 *  Created on: 25. 9. 2018
 *  Author:     Priesol Vladimir
 */


#include <rtc_f1.h>
#include "app.h"
#include "timer.h"

#define SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk
#define SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk

// spinani 5V ke zdroji 24V
#define SUPPLY24_PORT      GPIOB
#define SUPPLY24_PIN       GPIO_Pin_13

// EN pin L268
#define EN_PORT            GPIOA
#define EN_PIN             GPIO_Pin_1

// IN1 pin L268
#define IN1_PORT           GPIOA
#define IN1_PIN            GPIO_Pin_3

// IN2 pin L268
#define IN2_PORT           GPIOA
#define IN2_PIN            GPIO_Pin_2

// externi LED (blikani 1s)
#define LED_PORT           GPIOB
#define LED_PIN            GPIO_Pin_6

// tlacitko HODINY
#define HOUR_PORT          GPIOA
#define HOUR_PIN           GPIO_Pin_5

// tlacitko MINUTY
#define MIN_PORT           GPIOA
#define MIN_PIN            GPIO_Pin_4

// vstup ADC pro mereni napajeni 5V
#define SUPPLY_PORT        GPIOA
#define SUPPLY_PIN         GPIO_Pin_7

// BLUE LED na kitu
#define BOARD_LED_PORT     GPIOC
#define BOARD_LED_PIN      GPIO_Pin_13

#define WEAKUP_POWER_S           1
#define WEAKUP_BAT_S            60
#define IMPULSE_LENGTH_MS       500


typedef enum
{
  mode_power = 0,
  mode_bat,
} mode_t;

bool     g_bLastMinutePosition = false;
mode_t   g_eMode;
uint8_t  g_nSeconds = 0;
uint32_t g_nImpulseStack = 0;

void App_Init(void)
{
  Timer_Init();

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;

  // configure output pins
  GPIO_InitStruct.GPIO_Pin   = EN_PIN;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(EN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = IN1_PIN;
  GPIO_Init(IN1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = IN2_PIN;
  GPIO_Init(IN2_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = LED_PIN;
  GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = SUPPLY24_PIN;
  GPIO_Init(SUPPLY24_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = BOARD_LED_PIN;
  GPIO_Init(BOARD_LED_PORT, &GPIO_InitStruct);

  // configure input pins
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;

  GPIO_InitStruct.GPIO_Pin = HOUR_PIN;
  GPIO_Init(HOUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = MIN_PIN;
  GPIO_Init(MIN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = SUPPLY_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SUPPLY_PORT, &GPIO_InitStruct);

  GPIO_ResetBits(EN_PORT, EN_PIN);

  App_MinuteImpulse();
  App_MinuteImpulse();
  App_MinuteImpulse();
  App_MinuteImpulse();
  App_MinuteImpulse();
  App_MinuteImpulse();


  App_BoardLed(true);

  RTCF1_Init();
  Timer_SetSysTickCallback(App_SystickCallback);

//  RTCF1_SetWakeUp(5);

//  // EXTI configuration for RTC wakeup line
//  EXTI->IMR |= SUPPLY_PIN; // Configure the corresponding mask bit in the EXTI_IMR register
//  EXTI->RTSR |= SUPPLY_PIN; // Configure the Trigger Selection bits of the Interrupt line (rising edge)
//
//  NVIC_SetPriority(EXTI9_5_IRQn, 2);
//  NVIC_EnableIRQ(EXTI9_5_IRQn);


//  if (GPIO_ReadInputDataBit(SUPPLY_PORT, SUPPLY_PIN))
//  {
//    g_eMode = mode_power;
//    RTCF1_SetWakeUp(WEAKUP_POWER_S);
//  }
//  else
//  {
//    g_eMode = mode_bat;
//    RTCF1_SetWakeUp(WEAKUP_BAT_S);
//  }

}

void App_Exec(void)
{
  App_BoardLed(true);
  Timer_Delay_ms(500);
  App_BoardLed(false);
  APP_SetStopmode();
  return;

  if (g_eMode == mode_power)
  {
    if (g_nImpulseStack)  // je neco v zasobniku impulsu?
    {
      App_MinuteImpulse();
      g_nImpulseStack--;
    }

    if (!(RTC->CRL & RTC_CRL_SECF))
    {
      return;
    }

    // clear SEC flag
    RTC->CRL &= ~RTC_CRL_SECF;

    g_nSeconds++;
    if (g_nSeconds >= 60)
    {
      g_nSeconds = 0;
      if (GPIO_ReadInputDataBit(SUPPLY_PORT, SUPPLY_PIN))
      {
        App_MinuteImpulse();
      }
      else
      {
        // ztrata napajeni, prejit do BAT modu
//        g_eMode = mode_bat;
//        RTCF1_SetWakeUp(WEAKUP_BAT_S);
//        NVIC_EnableIRQ(RTC_IRQn);
      }
    }

  }

//  if (g_eMode == mode_bat)
//  {
//    g_nImpulseStack++;
//    APP_SetLPmode(false);
//  }

}

void App_MinuteImpulse()
{
  GPIO_SetBits(SUPPLY24_PORT, SUPPLY24_PIN);
  if (g_bLastMinutePosition)
  {
    GPIO_SetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
  }
  else
  {
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_SetBits(IN2_PORT, IN2_PIN);
  }

  Timer_Delay_ms(1);
  GPIO_SetBits(EN_PORT, EN_PIN);
  Timer_Delay_ms(IMPULSE_LENGTH_MS);
  GPIO_ResetBits(EN_PORT, EN_PIN);

  GPIO_ResetBits(SUPPLY24_PORT, SUPPLY24_PIN);

  g_bLastMinutePosition = !g_bLastMinutePosition;
}

void App_BoardLed(bool bEnable)
{
  if (bEnable)
  {
    GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN);
  }
  else
  {
    GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN);
  }

}

// STOP mode continues after 'wfi' instruction
// STANDBY mode continues from RESET vector and RAM is erased
void APP_SetStopmode()
{
  /* Wichtig! Löschen der Pending IRQ Flags */
  EXTI->PR = 0xFFFFFFFF;
  PWR_ClearFlag(PWR_FLAG_WU);
  SYSTICK_ISR_OFF;

  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

  SYSTICK_ISR_ON;
//  SystemInit(); // po probuzeni jsou hodiny vynulovany (PLL  a delicky)
}

void App_SystickCallback(void)
{
  // kontrola tlacitek
}

void EXTI4_15_IRQHandler(void)
{
//  // EXTI line interrupt detected
//  if (EXTI->PR & SUPPLY_PIN)
//  {
//    EXTI->PR = SUPPLY_PIN; // Clear interrupt flag
//
//    NVIC_DisableIRQ(RTC_IRQn);
//
//    // precist zbytkovy cas RTC->WUTR a nastavit podle neho sekundy
//    uint32_t nWutr = RTC->WUTR;
//    g_nSeconds = 59 - nWutr;
//    g_eMode = mode_power;
//    RTCF1_SetWakeUp(WEAKUP_POWER_S);
//  }
}
