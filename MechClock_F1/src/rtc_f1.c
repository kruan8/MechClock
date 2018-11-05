/*
 * rtc.c
 *
 *  Created on: 7. 11. 2016
 *  Author: Priesol Vladimir
 */

#include <rtc_f1.h>
#include "app.h"

/* Internal RTC defines */
#define RTC_LEAP_YEAR(year)             ((((year) % 4 == 0) && ((year) % 100 != 0)) || ((year) % 400 == 0))
#define RTC_DAYS_IN_YEAR(x)             RTC_LEAP_YEAR(x) ? 366 : 365
#define RTC_OFFSET_YEAR                 1970
#define RTC_SECONDS_PER_DAY             86400
#define RTC_SECONDS_PER_HOUR            3600
#define RTC_SECONDS_PER_MINUTE          60
#define RTC_BCD2BIN(x)                  ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define RTC_CHAR2NUM(x)                 ((x) - '0')
#define RTC_CHARISNUM(x)                ((x) >= '0' && (x) <= '9')

/* Days in a month */
static const uint8_t TM_RTC_Months[2][12] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}, /* Not leap year */
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}  /* Leap year */
};

uint32_t   g_nAlarmInterval;        // ulozeni intervalu alarmu pro pouziti v peruseni

void RTCF1_Init(void)
{
  RCC->APB1ENR |= RCC_APB1Periph_BKP | RCC_APB1Periph_PWR; // Enable BackUp & PWR clock
  PWR->CR |= PWR_CR_DBP; // Povolime si pristup do BKP

//  if (*(__IO uint16_t *)(((uint32_t)BKP_BASE)+BKP_DR10) != 0xA5A5)
//  {
    RCC->BDCR |= RCC_BDCR_BDRST; // reset BackUp  (nulovani backup registru)
    RCC->BDCR &= ~RCC_BDCR_BDRST;

    // RTC hodiny vychazeji z frekvence externiho oscilátoru 32 KHz
    RCC->BDCR |= RCC_LSE_ON; // povolit externi LSE oscilator
    while ((RCC->BDCR & RCC_BDCR_LSERDY) == RESET); // pockame na spusteni externiho oscilatoru 32kHz

    RCC->BDCR |= RCC_RTCCLKSource_LSE; // vybrat externi LSE oscilator
    RCC->BDCR |= RCC_BDCR_RTCEN; // povolit RTC
    RTC->CRL &= (uint16_t)~RTC_FLAG_RSF; // Cekej na synchronizaci
    while ((RTC->CRL & RTC_FLAG_RSF) == RESET);

    while ((RTC->CRL & RTC_FLAG_RTOFF) == RESET); // Pockame na dokonceni zapisu

    RTC->CRL |= RTC_CRL_CNF; // povolit nastaveni preddelicky RTC
    RTC->PRLH = 0; // (32767 & 0xF0000) >> 16; // nastaveni preddelicky RTC
    RTC->PRLL = 0x7FFF; // (32767 & 0xFFFF);
    RTC->CRL &= ~RTC_CRL_CNF; // zakaz nastaveni preddelicky RTC
    while ((RTC->CRL & RTC_FLAG_RTOFF) == RESET); // Pockame na dokonceni zapisu

//    *(__IO uint32_t *)(((uint32_t)BKP_BASE)+BKP_DR10) = 0xA5A5; // nastaveni priznaku nastaveni RTC
//  }

  // Nastavime typ alarmu
//  RTC->CRH |= RTC_IT_SEC; // Preruseni po 1 sek
  while ((RTC->CRL & RTC_FLAG_RTOFF) == RESET); // Pockame na dokonceni zapisu

  PWR->CR &= ~PWR_CR_DBP; // zakaz pristupu do BKP registru
}

/// Nasatvi alarm a INT
/// nInterval_s = 0 zakaze preruseni
void RTCF1_SetWakeUp(uint16_t nInterval_s)
{
  if (nInterval_s == 0)
  {
    RTC_ITConfig(RTC_IT_ALR, DISABLE);
    return;
  }

  // INT vyvolano na konci cyklu, kdy hodnota RTC-ALARM je shodna
  // pri nacitani RTC_CNT je uz hodnota o 1 vyssi
  nInterval_s--;

  EXTI_InitTypeDef    EXTI_InitStructure;
  NVIC_InitTypeDef    NVIC_InitStructure;

  /* EXTI configuration */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Alarmperiode setzen */
  PWR_BackupAccessCmd(ENABLE);
  RTC_WaitForLastTask();
  uint32_t cnt = RTC_GetCounter();
  RTC_WaitForLastTask();
  RTC_SetAlarm(cnt + nInterval_s);   // sleeptime
  RTC_WaitForLastTask();

  /* Int zulassen */
  NVIC_SetPriority(RTCAlarm_IRQn, 13);
  NVIC_EnableIRQ(RTCAlarm_IRQn);
  RTC_ITConfig(RTC_IT_ALR, ENABLE);

  PWR_BackupAccessCmd(DISABLE);

  g_nAlarmInterval = nInterval_s;
}

uint32_t RTCF1_GetRemainingAlarm(void)
{
  uint32_t nValue = (RTC->ALRH << 16) + RTC->ALRL;
  nValue -= RTC_GetCounter();
  return nValue;
}

void RTCF1_Set(rtc_record_time_t *dt)
{
  uint32_t nValue = RTCF1_GetUnixTimeStamp(dt);
  RTC_SetCounter(nValue);
}

void RTCF1_Get(rtc_record_time_t *dt)
{
  uint32_t nValue = RTC_GetCounter();
  RTCF1_GetDateTimeFromUnix(dt, nValue);
}

/// Get UNIX format  (number seconds from 1.1.1970) from YMD time
int32_t RTCF1_GetUnixTimeStamp(rtc_record_time_t* data)
{
  uint32_t days = 0, seconds = 0;
  uint16_t i;
  uint16_t year = (uint16_t) (data->year + 2000);
  /* Year is below offset year */
  if (year < RTC_OFFSET_YEAR) {
    return 0;
  }
  /* Days in back years */
  for (i = RTC_OFFSET_YEAR; i < year; i++) {
    days += RTC_DAYS_IN_YEAR(i);
  }
  /* Days in current year */
  for (i = 1; i < data->month; i++) {
    days += TM_RTC_Months[RTC_LEAP_YEAR(year)][i - 1];
  }
  /* Day starts with 1 */
  days += data->day - 1;
  seconds = days * RTC_SECONDS_PER_DAY;
  seconds += data->hour * RTC_SECONDS_PER_HOUR;
  seconds += data->min * RTC_SECONDS_PER_MINUTE;
  seconds += data->sec;

  /* seconds = days * 86400; */
  return seconds;
}

/// Get YMD time from unix format (number seconds from 1.1.1970)
void RTCF1_GetDateTimeFromUnix(rtc_record_time_t* data, uint32_t unix)
{
  uint16_t year;

  /* Get seconds from unix */
  data->sec = unix % 60;
  /* Go to minutes */
  unix /= 60;
  /* Get minutes */
  data->min = unix % 60;
  /* Go to hours */
  unix /= 60;
  /* Get hours */
  data->hour = unix % 24;
  /* Go to days */
  unix /= 24;

  /* Get week day */
  /* Monday is day one */
  data->day = (unix + 3) % 7 + 1;

  /* Get year */
  year = 1970;
  while (1) {
    if (RTC_LEAP_YEAR(year)) {
      if (unix >= 366) {
        unix -= 366;
      } else {
        break;
      }
    } else if (unix >= 365) {
      unix -= 365;
    } else {
      break;
    }
    year++;
  }
  /* Get year in xx format */
  data->year = (uint8_t) (year - 2000);
  /* Get month */
  for (data->month = 0; data->month < 12; data->month++) {
    if (RTC_LEAP_YEAR(year)) {
      if (unix >= (uint32_t)TM_RTC_Months[1][data->month]) {
        unix -= TM_RTC_Months[1][data->month];
      } else {
        break;
      }
    } else if (unix >= (uint32_t)TM_RTC_Months[0][data->month]) {
      unix -= TM_RTC_Months[0][data->month];
    } else {
      break;
    }
  }
  /* Get month */
  /* Month starts with 1 */
  data->month++;
  /* Get date */
  /* Date starts with 1 */
  data->day = unix + 1;
}

uint8_t RTCF1_ByteToBcd2(uint8_t Value)
{
  uint8_t bcdhigh = 0;

  while (Value >= 10)
  {
    bcdhigh++;
    Value -= 10;
  }

  return  ((uint8_t)(bcdhigh << 4) | Value);
}

/**
  * @brief  Convert from 2 digit BCD to Binary.
  * @param  Value: BCD value to be converted.
  * @retval Converted word
  */
uint8_t RTCF1_Bcd2ToByte(uint8_t Value)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}

void RTC_IRQHandler(void)
{
//  RTC->CRL &= ~RTC_CRL_SECF;

}

void RTCAlarm_IRQHandler ()
{
  if (RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
    PWR_BackupAccessCmd(ENABLE);

    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    uint32_t val = RTC_GetCounter();
    RTC_WaitForLastTask();
    RTC_SetAlarm(val + g_nAlarmInterval);
    RTC_WaitForLastTask();
    RTC_ClearITPendingBit(RTC_IT_ALR);
    PWR_BackupAccessCmd(DISABLE);

    EXTI_ClearITPendingBit(EXTI_Line17);   // Remove LINE interrupt flag bit
  }

}

void RTCF1_Test(void)
{
  // ---------- Test RTC ---------------
  rtc_record_time_t dt;
  rtc_record_time_t dt_new;

  dt.day = 15;
  dt.month = 11;
  dt.year = 16;
  dt.hour = 18;
  dt.min = 25;
  dt.sec = 0;
  RTCF1_Set(&dt);
  RTCF1_Get(&dt_new);
  uint32_t t = RTCF1_GetUnixTimeStamp(&dt);
  RTCF1_GetDateTimeFromUnix(&dt, t);
}
