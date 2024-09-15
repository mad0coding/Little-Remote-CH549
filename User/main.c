/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : L&E
* Version            : V1.0
* Date               : 2023/10/26
* Description        : Little-Remote
********************************************************************************/
#include "CH549.H"
#include "DEBUG.H"
#include "GPIO.H"
#include "ADC.H"
#include "PWM.H"
#include "Timer.H"
#include "drv_RF24L01.h"
#include "tools.h"
#include "XboxUSB.h"
#include "HidUSB.h"
#include "para.h"

#pragma  NOAREGS

uint8_t nrfFlag = 0, NRF_connect = 0;
uint8_t nrf_tick = 10, nrf_state = 1;//NRF发送节拍和状态
uint8_t ifNrfSend = 0;//0为不发送,1为需要发送
uint8_t ifNrfConnect = 0;//与接收端是否连接
uint8_t sendCycle = 0;//发送周期选择

//uint8_t loopCount = 0;//循环计数

uint8_t activeState = 0x01;//bit7为当前是否有操作 bit0为当前是否正常态

uint8_t printBuf[64];

uint8_t i = 0;

void main()
{
	CTRL = 0;								//上电先拉低上电控制 保留掉电机会
    CfgFsys( );								//CH549时钟选择配置
    mDelaymS(5);							//修改主频等待内部晶振稳定 必加
    //mInitSTDIO( );							//串口0初始化
    //printf("Little-Remote start.\n");
	
	GPIO_Init(PORT3, PIN7, MODE0);			//KP_0浮空输入
	CTRL = 1;								//拉高上电控制维持供电
	
	CH549WDTModeSelect(1);		//启动看门狗
	
	SetPWMClkDiv(32);						//PWM时钟分频配置,FREQ_SYS/32 = 768kHz
    SetPWMCycle256Clk();					//PWM周期,FREQ_SYS/32/256 = 3kHz
    SetPWM4Dat(255);						//初始占空比配置
	SetPWM5Dat(255);						//初始占空比配置
	PWM_SEL_CHANNEL(PWM_CH4, Enable);		//启动通道4输出使能
	PWM_SEL_CHANNEL(PWM_CH5, Enable);		//启动通道5输出使能
	
	ADC_ExInit(0);				//ADC初始化,选择最慢采样时钟
	ADC_ChSelect(ADCRx);		//选择通道0(Rx)
	ADC_ChSelect(ADCRy);		//选择通道1(Ry)
	ADC_ChSelect(ADCLy);		//选择通道2(Ly)
	ADC_ChSelect(ADCLx);		//选择通道3(Lx)
	ADC_ChSelect(ADCIN);		//选择通道13(VIN)
	ADC_StartSample();			//启动采样
	
	IT0 = 1;				//INT0下降沿触发
	EX0 = 1;				//允许INT0中断(KL按键)
	
	IT1 = 1;				//INT1下降沿触发
	EX1 = 1;				//允许INT1中断(KR按键)
	
	INTX |= bIX3 | bIT3;	//选择高电平和边沿触发(即上升沿触发)
	IE_INT3 = 1;			//允许INT3中断(电源按键)
	
//	mTimer0Clk12DivFsys();			//T0定时器时钟设置 FREQ_SYS/12
//	mTimer_x_ModInit(0, 1);			//T0定时器模式设置 模式1 16位定时器
//	mTimer_x_SetData(0, 0);			//T0定时器赋值 一直16位循环 不使用中断
//	mTimer0RunCTL(1);				//T0定时器启动
////	ET0 = 1;						//T0定时器中断开启

	mTimer2Clk12DivFsys();			//T2定时器时钟设置 FREQ_SYS/12
	mTimer_x_ModInit(2, 1);			//T2定时器模式设置 模式1 16位定时器
	mTimer_x_SetData(2, 0);			//T2定时器赋值 一直16位循环
	mTimer2RunCTL(1);				//T2定时器启动
	ET2 = 1;						//T2定时器中断开启
	
	if(0) SYSMOD = 1;//测试代码
	else SYSMOD = 2;
	
	if(SYSMOD == 1){		//Xbox模式
		UsbXboxInit();						//Xbox USB设备初始化
	}
	else if(SYSMOD == 2){	//HID模式
		UsbDeviceInit();					//HID USB设备初始化
	}
	
	memset(XboxData,0,sizeof(XboxData));	//清空缓冲区
//    USBDeviceInit();						//USB设备模式初始化
    EA = 1;									//允许单片机中断
	
	SPIMasterModeSet(0);			//SPI主机模式设置(模式0)
    SPI_CK_SET(3);					//3分频(24/3=8M,NRF最高10M速率)
	NRF24L01_Config(0);				//初始化为发送

	ifNrfSend = 1;
	
	//for(i = 0; i < 64; i++) FlashBuf[i] = i + 1;
	
	//ParaSave(0, 1);//参数保存
	
	memcpy(printBuf, FlashBuf, 64);
	
	while(1){
		Enp2IntIn(printBuf, 64);
		mDelaymS(500);
		WDOG_COUNT = 0;//清零看门狗计数
	}
	
    while(1)
    {
		T2COUNT;
		IO_read();
		
		//IO_use[0];
		XboxData[0] = 0; XboxData[1] = 20;
		XboxData[2] = 0;
		XboxData[2] |= (uint8_t)(!KP_1) << 3 | (uint8_t)(!KP_3) << 2 | (uint8_t)(!KP_4) << 1 | (uint8_t)(!KP_2) << 0;
		XboxData[3] = 0;
		XboxData[3] |= (uint8_t)(!KP_6) << 7 | (uint8_t)(!KP_7) << 6 | (uint8_t)(!KP_5) << 5 | (uint8_t)(!KP_8) << 4;
		XboxData[3] |= (uint8_t)(!KP_L) << 2 | (0) << 1 | (0);

//		XboxData[4] = IO_RK[0] >> 8;
//		XboxData[5] = IO_RK[1] >> 8;
		
		XboxData[6] = -IO_RK[0];	XboxData[7] = -IO_RK[0] >> 8;
		XboxData[8] = IO_RK[1];		XboxData[9] = IO_RK[1] >> 8;
		XboxData[10] = -IO_RK[2];	XboxData[11] = -IO_RK[2] >> 8;
		XboxData[12] = IO_RK[3];	XboxData[13] = IO_RK[3] >> 8;
		
		Enp1XboxIn(XboxData, 20);
		
		if(nrf_tick > 0){//暂不需发送
			nrf_tick--;
		}
		if(nrf_tick == 0 && nrf_state == 1 && ifNrfSend == 1){//需要发送
			nrf_tick = sendCycle * 2 + 1;//发送周期(sendCycle*2+1+1)*5ms
			nrf_state = 2;//进入等待发送完成
			IO_pack();//数据打包
			NRF24L01_TxStart((uint8_t*)STD_pkg, 16);//启动发送
//			if(pkgSelect == 0) NRF24L01_TxStart( (uint8_t *)STD_pkg, 16 );
//			else if(pkgSelect == 1) NRF24L01_TxStart( (uint8_t *)EXTD_pkg, 24 );
//			else if(pkgSelect == 2) NRF24L01_TxStart( (uint8_t *)SIMP_pkg, 10 );
		}
		else if(nrf_state == 2){//等待发送完成
			uint8_t nrfReturn = NRF24L01_TxCheck();//检查是否发送完成
			if(nrfReturn != 0xFD){//若不是仍在等待发送完成
				nrf_state = 1;//再次进入需要发送
			}
			if(nrfReturn == TX_OK){//若发送成功
				ifNrfConnect = 20 - sendCycle * 2;//连接强度补满
//				TIM2->CCR1 = 0;
			}
			else if(nrfReturn == MAX_TX || nrfReturn == 0xFE){//重发次数或检查次数用尽
				if(ifNrfConnect) ifNrfConnect--;//连接强度递减
//				TIM2->CCR1 = 1000;
			}
			if(ifNrfSend) SET_PWM_RF(ifNrfConnect * 10);//绿LED表示连接强度
		}
		
		if(ActiveCheck()) activeState |= 0x80;//检测是否活动
		ifNrfSend = activeState & 0x01;//正常态才发送
		if(!ifNrfSend) SET_PWM_RF(0);//绿LED表示连接强度
		
		WDOG_COUNT = 0;//清零看门狗计数
//		loopCount = (loopCount + 1) % 160;//循环周期计数
		mDelaymS(5);
    }
}

void INT_NO_INT0_ISR(void) interrupt INT_NO_INT0//KL按下
{
	static uint8_t state = 0;
	SET_PWM_LED2(state);
	SetPWM4Dat(state);
	state = 255 - state;
}

void INT_NO_INT1_ISR(void) interrupt INT_NO_INT1//KR按下 测试代码
{
	SET_PWM_LED2(255);//蓝LED熄灭
	CTRL = 0;//为保证测试顺利 预留强关机
	while(1) WDOG_COUNT = 0;//清零看门狗计数
}

void INT_NO_INT3_ISR(void) interrupt INT_NO_INT3//K0按下
{
	if(0){
		
	}
}

void mTimer2Interrupt(void) interrupt INT_NO_TMR2//Timer2中断服务程序
{
	static uint8_t div = 0, K0count = 255, BatLvCount = 0;//分频值 计数值 计数值
	static uint16_t noActCount = 0;//非活动计数
	div++;//32.768ms中断一次 约30次/s
	if(div >= 3) div = 0;//3分频 并在以下分3个相位完成工作以免集中占用时间
	if(div == 0){//约10次/s 相位0 处理电源按键
		if(!KP_0) K0count = 0;//K0抬起则清零计数值
		else if(K0count != 255){//K0按下且非启动时的按下
			K0count++;//向上计数
			if(K0count >= 15){//到达设定长按时间(单位0.1s)
				SET_PWM_POWER(0);//上电指示LED熄灭(蓝)
				CTRL = 0;//预关机(按键松开后掉电)
				while(1) WDOG_COUNT = 0;//清零看门狗计数
			}
		}
	}
	else if(div == 1){//约10次/s 相位1 处理低电量
		BatLvCount++;//98.304ms一次
		if(BatLvCount >= 10) BatLvCount = 0;//16分频
		if(AdcValue[4] < ADC_3V6/* || !KP_LL*/){//电量低于3.6V
			if(AdcValue[4] < ADC_3V4/* || !KP_RR*/){//电量低于3.4V
				for(BatLvCount = 0; BatLvCount < 10; BatLvCount++){//借用作计数变量
					SET_PWM_POWER(1);	mDelaymS(100);
					SET_PWM_POWER(0);	mDelaymS(100);
					WDOG_COUNT = 0;//清零看门狗计数
				}
				CTRL = 0;//关机
				while(1) WDOG_COUNT = 0;//清零看门狗计数
			}
			else if(AdcValue[4] < ADC_3V5/* || !KP_LR*/){//3.4V与3.5V之间
				SET_PWM_POWER(BatLvCount < 2 || (BatLvCount >=5 && BatLvCount < 7));//0.5s周期40%占空比
			}
			else{//3.5V与3.6V之间
				SET_PWM_POWER(BatLvCount < 5);//1s周期50%占空比
			}
		}
		else SET_PWM_POWER(1);//非低电
	}
	else if(div == 2){//约10次/s 相位2 处理长期无操作
		if(activeState & 0x80){//若有操作
			activeState &= ~0x80;//清除活动标志
			noActCount = 0;//非活动计数清零
			activeState |= 0x01;//取消待机进入正常态
		}
		else{//若无操作
			noActCount++;//非活动计数递增
			if(activeState & 0x01){//正常态 即第一轮
				if(noActCount > 10*60*10){//第一轮计数到期
					noActCount = 0;//非活动计数清零 准备第二轮计数
					activeState &= ~0x01;//进入非发送待机态
//					SET_PWM_POWER(0);	mDelaymS(100); //测试代码
				}
			}
			else{//待机态 即第二轮
				if(noActCount > 10*60*10){//第二轮计数到期
					for(noActCount = 0; noActCount < 10; noActCount++){//借用作计数变量
						SET_PWM_POWER(1);	mDelaymS(100);
						SET_PWM_POWER(0);	mDelaymS(100);
						WDOG_COUNT = 0;//清零看门狗计数
					}
					CTRL = 0;//关机
					while(1) WDOG_COUNT = 0;//清零看门狗计数
				}
			}
		}
	}
	TF2 = 0;//需手动清中断标志
}





