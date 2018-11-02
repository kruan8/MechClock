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

// OUTPUTS definition

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

// externi LED (blink 1s)
#define LED_PORT           GPIOB
#define LED_PIN            GPIO_Pin_6

// BLUE LED na kitu
#define BOARD_LED_PORT     GPIOC
#define BOARD_LED_PIN      GPIO_Pin_13


// INPUTS definition

// tlacitko HODINY
#define KEY_HOUR_PORT      GPIOA
#define KEY_HOUR_PIN       GPIO_Pin_5

// tlacitko MINUTY
#define KEY_IN_PORT        GPIOA
#define KEY_MIN_PIN        GPIO_Pin_4

// vstup ADC pro mereni napajeni 5V
#define SUPPLY_PORT        GPIOA
#define SUPPLY_PIN         GPIO_Pin_7


// port control
#define LED_ON             LED_PORT->BRR = LED_PIN
#define LED_OFF            LED_PORT->BSRR = LED_PIN

#define SUPPLY24_ON        SUPPLY24_PORT->BSRR = SUPPLY24_PIN
#define SUPPLY24_OFF       SUPPLY24_PORT->BRR = SUPPLY24_PIN

#define IMPULSE_ON         EN_PORT->BSRR = EN_PIN
#define IMPULSE_OFF        EN_PORT->BRR = EN_PIN

#define IN1_ON             IN1_PORT->BSRR = IN1_PIN
#define IN1_OFF            IN1_PORT->BRR = IN1_PIN

#define IN2_ON             IN2_PORT->BSRR = IN2_PIN
#define IN2_OFF            IN2_PORT->BRR = IN2_PIN

#define BOARD_LED_ON       BOARD_LED_PORT->BSRR = BOARD_LED_PIN
#define BOARD_LED_OFF      BOARD_LED_PORT->BRR = BOARD_LED_PIN

#define APP_WEAKUP_POWER_S           1
#define APP_WEAKUP_BAT_S            60

#define APP_DELAY_24V_MS            50     // cekani na nabehnuti zdroje 24V
#define APP_IMPULSE_LENGTH_MS      500     // delka impulsu

// keys
#define APP_KEY_HOUR           1 << 0
#define APP_KEY_MIN            1 << 1
#define APP_KEY_DEBOUNCING     20

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

  GPIO_InitStruct.GPIO_Pin = KEY_HOUR_PIN;
  GPIO_Init(KEY_HOUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = KEY_MIN_PIN;
  GPIO_Init(KEY_IN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = SUPPLY_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SUPPLY_PORT, &GPIO_InitStruct);

  IMPULSE_OFF;

  // Todo: funguje to vubec?
  BOARD_LED_ON;

  RTCF1_Init();
  Timer_SetSysTickCallback(App_SystickCallbackINT);

//  RTCF1_SetWakeUp(5);

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

  g_eMode = mode_power;
  g_nImpulseStack += 10;

}

void App_Exec(void)
{
  // v BAT modu prejit do STOP modu a pak pridat impulz
  if (g_eMode == mode_bat)
  {
    APP_SetStopMode();
    g_nImpulseStack++;
    return;
  }

  // cisteni zasobniku impulsu
  // pokud neni impuls v zaspobniku, reagujeme na tlacitka
  if (g_nImpulseStack)
  {
    App_MinuteImpulse();
    g_nImpulseStack--;
  }
  else
  {
    // obsluha tlacitek
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
  if (RTC->CRL & RTC_CRL_SECF)
  {
    // clear SEC flag
    RTC->CRL &= ~RTC_CRL_SECF;

    g_nSeconds++;
    if (g_nSeconds >= 60)
    {
      g_nSeconds = 0;
      g_nImpulseStack++;
    }
  }

  // pokud je ztrata napajeni, prejit do BAT modu
  if (!(SUPPLY_PORT->IDR & SUPPLY_PIN))
  {
    // Todo: debouncing, neni kam spechat
    //  g_eMode = mode_bat;
    //  RTCF1_SetWakeUp(WEAKUP_BAT_S);
    //  NVIC_EnableIRQ(RTC_IRQn);
  }

}

void App_MinuteImpulse()
{
  // zapnout zdroj 24V
  SUPPLY24_ON;

  // wait for supply startup
  Timer_Delay_ms(APP_DELAY_24V_MS);

  if (g_bAnchorPosition)
  {
    IN1_ON;
    IN2_OFF;
  }
  else
  {
    IN1_OFF;
    IN2_ON;
  }

  // impuls do civky + LED bliknuti
  IMPULSE_ON;
  LED_ON;
  Timer_Delay_ms(APP_IMPULSE_LENGTH_MS);
  LED_OFF;
  IMPULSE_OFF;

  g_bAnchorPosition = !g_bAnchorPosition;

  // pokud je v zasobniku impuls, nevypinat napajeni zdroje 24V (vykonnova zatez spinace)
  if (!g_nImpulseStack)
  {
    SUPPLY24_OFF;
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
//  SystemInit(); // po probuzeni jsou hodiny vynulovany (PLL  a delicky) - jak na to prisli???

}

void App_SystickCallbackINT(void)
{
  static uint8_t nKeyCounter = 0;
  static uint8_t g_nLastKeys;

  // snimani stavu tlacitek
  uint32_t nKeys = 0;
  if (KEY_HOUR_PORT->IDR & KEY_HOUR_PIN)
  {
    nKeys |= APP_KEY_HOUR;
  }

  if (KEY_IN_PORT->IDR & KEY_MIN_PIN)
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
