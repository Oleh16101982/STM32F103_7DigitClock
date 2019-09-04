#ifndef RTC_H_
#include "stm32f10x_conf.h"
//#include <stdint.h>
//#include <stdbool.h>

#define FIRSTYEAR   2000		// start year
#define FIRSTDAY    6			// 0 = Sunday

typedef struct {
	uint16_t year;	/* 1..4095 */
	uint8_t  month;	/* 1..12 */
	uint8_t  mday;	/* 1.. 31 */
	uint8_t  wday;	/* 0..6, Sunday = 0*/
	uint8_t  hour;	/* 0..23 */
	uint8_t  min;	/* 0..59 */
	uint8_t  sec;	/* 0..59 */
	uint8_t  dst;	/* 0 Winter, !=0 Summer */
} RTC_t;

typedef struct {
	uint8_t year;	/* 00..99 */
	uint8_t  month;	/* 01..12 */
	uint8_t  mday;	/* 01.. 31 */
	uint8_t  wday;	/* 0..6, Sunday = 0*/
	uint8_t  hour;	/* 00..23 */
	uint8_t  min;	/* 00..59 */
	uint8_t  sec;	/* 00..59 */
	uint8_t  dst;	/* 0 Winter, !=0 Summer */
} RTC_t_BCD;

int rtc_init(void);
void rtc_gettime (RTC_t*);					/* Get time */
void rtc_gettimeBCD(RTC_t *rtc, RTC_t_BCD *rtcBCD);
void rtc_settime (const RTC_t*);				/* Set time */


#endif
