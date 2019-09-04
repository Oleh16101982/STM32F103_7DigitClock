#include "stm32f10x_conf.h"
#include "main.h"
#include "AlarmBcdValue.h"

extern uint16_t BKPDataReg[BKP_DR_NUMBER];
extern uint16_t bcd_alarm[2];

void DefineAlarmBcdValue(uint8_t numberAlarm)
{
	bcd_alarm[numberAlarm] = BKP_ReadBackupRegister(BKPDataReg[numberAlarm + 1]);
}
