
#include "rtc.h"
#include "bcdhex.h"

extern uint16_t bcd_hour_minutes;
extern uint16_t bcd_minutes_second;
extern uint16_t bcd_day_month;
extern uint16_t bcd_year;

RTC_t rtc;
RTC_t_BCD rtcBCD;
// RTC_t rtcprev;  // check if need uncomment
// RTC_t_BCD rtcBCDprev;

void DefineRtcBcdValue(void)
{
//	rtcprev = rtc; // check if need uncomment
//	rtcBCDprev = rtcBCD;
	rtc_gettime(&rtc);
	
	rtc_gettimeBCD(&rtc, &rtcBCD);	
	bcd_hour_minutes		= (rtcBCD.hour << 8) + rtcBCD.min;
	bcd_minutes_second	= (rtcBCD.min << 8) + rtcBCD.sec;
	bcd_day_month				= (rtcBCD.mday << 8) + rtcBCD.month; 
	bcd_year						= HEX16_to_BCD(rtc.year);
	
/*	
// variant 2	
	rtc_gettimeBCD(&rtc, &rtcBCD);	
	bcd_hour_minutes		= (HEX8_to_BCD(rtc.hour) << 8) + HEX8_to_BCD(rtc.min);
	bcd_minutes_second	= (HEX8_to_BCD(rtc.min) << 8) + HEX8_to_BCD(rtc.sec);
	BCD_DAY_MONTH				= (HEX8_to_BCD(rtc.mday) << 8) + HEX8_to_BCD(rtc.month);
	BCD_YEAR						= HEX16_to_BCD(rtc.year);	
*/	
}
