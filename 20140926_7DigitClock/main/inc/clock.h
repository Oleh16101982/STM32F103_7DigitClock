#include "stm32f10x_conf.h"

struct Time_s
{
  uint8_t SecLow;
  uint8_t SecHigh;
  uint8_t MinLow;
  uint8_t MinHigh;
  uint8_t HourLow;
  uint8_t HourHigh;
};
extern struct Time_s s_TimeStructVar;

/* Alarm Structure definition */
struct AlarmTime_s
{
  uint8_t SecLow;
  uint8_t SecHigh;
  uint8_t MinLow;
  uint8_t MinHigh;
  uint8_t HourLow;
  uint8_t HourHigh;
};
extern struct AlarmTime_s s_AlarmStructVar;

/* Date Structure definition */
struct Date_s
{
  uint8_t Month;
  uint8_t Day;
  uint16_t Year;
};
extern struct Date_s s_DateStructVar;

/* Alarm Date Structure definition */
struct AlarmDate_s
{
  uint8_t Month;
  uint8_t Day;
  uint16_t Year;
};
extern struct AlarmDate_s s_AlarmDateStructVar;


void CalculateTime(void);
