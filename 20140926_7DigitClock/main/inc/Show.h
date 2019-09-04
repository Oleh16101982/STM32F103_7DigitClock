
#include "stm32f10x_conf.h"

#define MAXANODE 4
#define MAXSEGMENT 8
#define MAXSEPARATOR 2
#define MAXDIGIT 10
#define MAXSTEPSPWM 0x40

// const static uint8_t arrPWM[MAXSTEPSPWM] = {0,3,4,6,9,11,14,16,18,21,23,26,29,32,35,38,41,45,48,52,55,59,63,66,70,74,78,82,84,86,91,95,99,104,108,113,115,117,122,126,131,136,141,146,151,156,161,166,172,177,182,187,193,198,204,210,215,221,227,232,238,244,250,255};
// another table
const static uint8_t arrPWM[MAXSTEPSPWM] = {0,1,2,3,4,5,6,6,6,7,7,8,8,9,9,10,11,12,12,13,14,15,16,17,19,20,21,23,24,26,28,30,32,34,36,39,42,44,48,51,54,58,62,66,71,76,81,87,93,100,106,114,122,130,139, 149,159,170,182,195,208,223,238,255};

const static	 uint8_t arrDigit[MAXDIGIT] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };	 
// const static uint8_t arrDigit[MAXDIGIT] = {0xc0, 0xf9, 0x5b, 0xd0, 0x99, 0x92, 0x82, 0xf8, 0x10, 0x90 };	



void defAnodeValue(uint8_t numberAnode);
void defSegmentValue(uint8_t numberAnode);
void def7DigitSegmentValueCurr(void);
void ChangeSeparatorMode(void);
void DefineSeparartorValue(void);
void defSetSeparatorValue(void);
void defSysTickSegmentValue(void);


