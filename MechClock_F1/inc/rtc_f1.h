/*
 * rtc.h
 *
 *  Created on: 7. 11. 2016
 *      Author: priesolv
 */

#ifndef RTC_H_
#define RTC_H_

#include <stdbool.h>
#include <stm32f10x.h>

typedef struct
{
  uint8_t second: 4;
  uint8_t second10: 4;
  uint8_t minute: 4;
  uint8_t minute10: 4;
  uint8_t hour: 4;
  uint8_t hour10: 4;
} rtc_time_t;

typedef struct
{
  uint8_t day: 4;
  uint8_t day10: 4;
  uint8_t month: 4;
  uint8_t month10: 1;
  uint8_t week_day: 3;
  uint8_t year: 4;
  uint8_t year10: 4;
} rtc_date_t;

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint16_t year;
}rtc_record_time_t;

void RTCF1_Init(void);
void RTCF1_Set(rtc_record_time_t *dt);
void RTCF1_Get(rtc_record_time_t *dt);
void RTCF1_SetWakeUp(uint16_t nInterval);
uint32_t RTCF1_GetRemainingAlarm(void);

void RTC_GetDT(rtc_time_t* time, rtc_date_t* date);

uint32_t RTC_GetTicks();
uint32_t RTC_GetUsartTimer();

void RTCF1_WriteBackup(uint32_t nBkpReg, uint32_t nData);
uint32_t RTCF1_ReadBackup(uint32_t nBkpReg);

int32_t RTCF1_GetUnixTimeStamp(rtc_record_time_t* data);
void RTCF1_GetDateTimeFromUnix(rtc_record_time_t* data, uint32_t unix);

uint8_t RTCF1_ByteToBcd2(uint8_t Value);
uint8_t RTCF1_Bcd2ToByte(uint8_t Value);

void RTCF1_Test(void);

#endif /* RTC_H_ */
