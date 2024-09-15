#ifndef _TOOLS_H
#define _TOOLS_H


#include "CH549.H"
#include "DEBUG.H"
#include "ADC.H"
#include "PWM.H"

#define SYSMOD	RESET_KEEP//用复位保持寄存器存储系统模式

//引脚定义
#define CTRL	P0_6

#define SW_1	P0_4
#define SW_2	P2_4

#define KP_L	P3_2
#define KP_R	P3_3

#define KP_0	P3_7
#define KP_1	P0_3
#define KP_2	P0_2
#define KP_3	P0_1
#define KP_4	P0_0
#define KP_5	P4_0
#define KP_6	P2_2
#define KP_7	P2_3
#define KP_8	P4_1

#define KP_LR	KP_1
#define KP_LU	KP_2
#define KP_LL	KP_3
#define KP_LD	KP_4
#define KP_RR	KP_5
#define KP_RU	KP_6
#define KP_RL	KP_7
#define KP_RD	KP_8

#define LED1	P2_0
#define LED2	P2_1
#define LED3_1	P3_4
#define LED3_2	P3_5
#define LED3_3	P3_6
#define LED3_4	P4_2
#define LED3_5	P4_3
#define LED3_6	P4_6

//ADC通道定义
#define ADCLx	CH3
#define ADCLy	CH2
#define ADCRx	CH0
#define ADCRy	CH1
#define ADCIN	CH13

#define Center_LX	2038//2023.11.8校正
#define Center_LY	1994
#define Center_RX	2048
#define Center_RY	2116
//2837 4.64
//2757 4.50
//2550 4.16
//y = 1.6665x - 90.63(x单位：ADC，y单位：mV)
//计算：2095,3.4;2155,3.5;2215,3.6
#define ADC_3V4		2095
#define ADC_3V5		2155
#define ADC_3V6		2215

//抽象LED操作API
#define SET_PWM_RF(x)		SET_PWM_LED1(x)//无线指示灯
#define SET_PWM_POWER(x)	SET_PWM_LED3(x)//上电指示灯

//物理LED操作API
#define SET_PWM_LED1(x)		(SetPWM5Dat(255 - (x)))//LED1 绿
#define SET_PWM_LED2(x)		(SetPWM4Dat(255 - (x)))//LED2 红
#define SET_PWM_LED3(x)		(LED3_1 = LED3_2 = LED3_3 = LED3_4 = LED3_5 = LED3_6 = ((x) ? 1 : 0))//LED3 蓝


extern uint16_t AdcValue[5];

extern uint8_t NRF_Rx_pkg[32];
extern uint8_t NRF_KEY[24];
extern uint8_t NRF_EC;
extern int16_t NRF_RK[4];

extern uint8_t STD_pkg[16];

extern int16_t IO_RK[4];
extern uint8_t IO_use[16];//使用IO输入


uint8_t NRF_Unpack(uint8_t CheckSum);

void IO_read(void);

uint8_t IO_pack(void);

uint8_t ActiveCheck(void);



#endif









