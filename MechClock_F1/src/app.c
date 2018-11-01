/*
 * app.c
 *
 *  Created on: 25. 9. 2018
 *  Author:     Priesol Vladimir
 */


#include <rtc_f1.h>
#include "app.h"
#include "timer.h"

#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

#define APP_KEY_HOUR           1 << 0
#define APP_KEY_MIN            1 << 1
#define APP_KEY_DEBOUNCING     20

#define SUPPLY24_PORT      GPIOB
#define SUPPLY24_PIN       GPIO_Pin_12

#define EN_PORT            GPIOB
#define EN_PIN             GPIO_Pin_13

#define IN1_PORT           GPIOB
#define IN1_PIN            GPIO_Pin_14

#define IN2_PORT           GPIOB
#define IN2_PIN            GPIO_Pin_15

#define LED_PORT           GPIOB
#define LED_PIN            GPIO_Pin_6

#define HOUR_PORT          GPIOA
#define HOUR_PIN           GPIO_Pin_3

#define MIN_PORT           GPIOA
#define MIN_PIN            GPIO_Pin_4

#define SUPPLY_PORT        GPIOA
#define SUPPLY_PIN         GPIO_Pin_2

#define BOARD_LED_PORT     GPIOC
#define BOARD_LED_PIN      GPIO_Pin_13

#define LED_ON             LED_PORT->BRR = LED_PIN
#define LED_OFF            LED_PORT->BSRR = LED_PIN

#define SUPPLY24_ON        SUPPLY24_PORT->BRR = SUPPLY24_PIN
#define SUPPLY24_OFF       SUPPLY24_PORT->BSRR = SUPPLY24_PIN

#define APP_WEAKUP_POWER_S           1
#define APP_WEAKUP_BAT_S            60

#define APP_DELAY_24V_MS            50     // cekani na nabehnuti zdroje 24V
#define APP_IMPULSE_LENGTH_MS      500     // delka impulsu


typedef enum
{
  mode_power = 0,
  mode_bat,
} mode_t;

bool     g_bAnchorPosition = false;
mode_t   g_eMode;
uint8_t  g_nSeconds = 0;
uint32_t g_nImpulseStack = 0;

uint8_t  g_nKey;

void App_Init(void)
{
  Timer_Init();

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;

  // configure output pins
  GPIO_InitStruct.GPIO_Pin   = EN_PIN;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
//  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;

  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(EN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = IN1_PIN;
  GPIO_Init(IN1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = IN2_PIN;
  GPIO_Init(IN2_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = LED_PIN;
  GPIO_Init(LED_PORT, &GPIO_InitStruct);

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

  App_BoardLed(true);

  RTCF1_Init();
  Timer_SetSysTickCallback(App_SystickCallbackINT);

  RTCF1_SetWakeUp(5);

//  // EXTI configuration for RTC wakeup line
//  EXTI->IMR |= SUPPLY_PIN; // Configure the corresponding mask bit in the EXTI_IMR register
//  EXTI->RTSR |= SUPPLY_PIN; // Configure the Trigger Selection bits of the Interrupt line (rising edge)
//
//  NVIC_SetPriority(EXTI9_5_IRQn, 2);
//  NVIC_EnableIRQ(EXTI9_5_IRQn);


  if (SUPPLY_PORT->IDR & SUPPLY_PIN)
  {
//    g_eMode = mode_power;
//    RTCF1_SetWakeUp(WEAKUP_POWER_S);
  }
  else
  {
//    g_eMode = mode_bat;
//    RTCF1_SetWakeUp(WEAKUP_BAT_S);
  }

}

void App_Exec(void)
{
  App_BoardLed(true);
  Timer_Delay_ms(500);
  App_BoardLed(false);
  APP_SetStopMode();
  return;

  if (g_eMode == mode_power)
  {
    // cisteni zasobniku impulsu
    // pokud neni impuls v zaspobniku, reagujeme na tlacitka
    if (g_nImpulseStack)
    {
      App_MinuteImpulse();
      g_nImpulseStack--;
    }
    else
    {
      // pridat minutovy impuls
      if (g_nKey & APP_KEY_MIN)
      {
        g_nImpulseStack--;
      }
      else if (g_nKey & APP_KEY_HOUR)
      {
        // pridat 60 minutovych impulsu
        g_nImpulseStack += 60;
      }
    }

    // kontrola uplynuti sekundy
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
      if (SUPPLY_PORT->IDR & SUPPLY_PIN)
      {
        App_MinuteImpulse();
      }
      else
      {
        // ztrata napajeni, prejit do BAT modu
        g_eMode = mode_bat;
        RTCF1_SetWakeUp(APP_WEAKUP_BAT_S);
      }
    }
  }

}

void App_MinuteImpulse()
{
  // zapnout zdroj 24V
  SUPPLY24_ON;
  Timer_Delay_ms(APP_DELAY_24V_MS);

  if (g_bAnchorPosition)
  {
    GPIO_SetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
  }
  else
  {
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_SetBits(IN2_PORT, IN2_PIN);
  }

  // impuls do civky
  GPIO_SetBits(EN_PORT, EN_PIN);
  Timer_Delay_ms(APP_IMPULSE_LENGTH_MS);
  GPIO_ResetBits(EN_PORT, EN_PIN);

  g_bAnchorPosition = !g_bAnchorPosition;
  SUPPLY24_OFF;
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
void APP_SetStopMode()
{
  /* Wichtig! Löschen der Pending IRQ Flags */
  EXTI->PR = 0xFFFFFFFF;
  PWR_ClearFlag(PWR_FLAG_WU);
  APP_SYSTICK_ISR_OFF;

  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

  APP_SYSTICK_ISR_ON;
//  SystemInit(); // po probuzeni jsou hodiny vynulovany (PLL  a delicky)

  g_nImpulseStack++;
}

void App_SystickCallbackINT(void)
{
  static uint8_t nKeyCounter = 0;
  static uint8_t g_nLastKeys;

  // snimani stavu tlacitek
  uint32_t nKeys = 0;
  if (HOUR_PORT->IDR & HOUR_PIN)
  {
    nKeys |= APP_KEY_HOUR;
  }

  if (MIN_PORT->IDR & MIN_PIN)
  {
    nKeys |= APP_KEY_MIN;
  }

  if (nKeys == g_nLastKeys)
  {
    nKeyCounter++;
    if (nKeyCounter > APP_KEY_DEBOUNCING)
    {
      nKeyCounter = APP_KEY_DEBOUNCING;
      g_nKey = nKeys;
    }
  }
  else
  {
    nKeyCounter = 0;
  }
}

void EXTI4_15_IRQHandler(void)
{
  /*
  // EXTI line interrupt detected
  if (EXTI->PR & SUPPLY_PIN)
  {
    EXTI->PR = SUPPLY_PIN; // Clear interrupt flag

    NVIC_DisableIRQ(RTC_IRQn);

    // precist zbytkovy cas RTC->WUTR a nastavit podle neho sekundy
    uint32_t nWutr = RTC->WUTR;
    g_nSeconds = 59 - nWutr;
    g_eMode = mode_power;
    RTCF1_SetWakeUp(WEAKUP_POWER_S);

    Todo: prepocitat impulsy v zasobniku na %den (60 minut * 24 hodin)
    g_nImpulseStack = g_nImpulseStack % (60 * 24);
  }
  */

}
