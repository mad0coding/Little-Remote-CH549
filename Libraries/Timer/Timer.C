/********************************** (C) COPYRIGHT *******************************
* File Name          : Timer.C
* Author             : WCH
* Version            : V1.0
* Date               : 2018/08/21
* Description        : CH549 Time 初始化、定时器、计数器赋值、T2捕捉功能开启函数等
                       定时器中断函数
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/
#include "Timer.H"
#pragma  NOAREGS

#ifdef  T2_CAP
UINT16 Cap2[2] = {0};
UINT16 Cap1[2] = {0};
UINT16 Cap0[2] = {0};
#endif

/*******************************************************************************
* Function Name  : mTimer_x_ModInit(UINT8 x ,UINT8 mode)
* Description    : CH549定时计数器x模式设置
* Input          : UINT8 mode,Timer模式选择
                   0：模式0，13位定时器，TLn的高3位无效
                   1：模式1，16位定时器
                   2：模式2，8位自动重装定时器
                   3：模式3，两个8位定时器  Timer0
                   3：模式3，Timer1停止
* Output         : None
* Return         : 成功  SUCCESS
                   失败  FAIL
*******************************************************************************/
UINT8 mTimer_x_ModInit(UINT8 x ,UINT8 mode)
{
    if(x == 0)
    {
        TMOD = TMOD & 0xf0 | mode;
    }
    else if(x == 1)
    {
        TMOD = TMOD & 0x0f | (mode<<4);
    }
    else if(x == 2)
    {
        RCLK = 0;    //16位自动重载定时器
        TCLK = 0;
        CP_RL2 = 0;
    }
    else
    {
        return FAIL;
    }
    return SUCCESS;
}
/*******************************************************************************
* Function Name  : mTimer_x_SetData(UINT8 x,UINT16 dat)
* Description    : CH549Timer0 TH0和TL0赋值
* Input          : UINT16 dat;定时器赋值
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer_x_SetData(UINT8 x,UINT16 dat)
{
    UINT16 tmp;
    tmp = 65536 - dat;
    if(x == 0)
    {
        TL0 = tmp & 0xff;
        TH0 = (tmp>>8) & 0xff;
    }
    else if(x == 1)
    {
        TL1 = tmp & 0xff;
        TH1 = (tmp>>8) & 0xff;
    }
    else if(x == 2)
    {
        RCAP2L = TL2 = tmp & 0xff;                                               //16位自动重载定时器
        RCAP2H = TH2 = (tmp>>8) & 0xff;
    }
}
/*******************************************************************************
* Function Name  : CAP2Init(UINT8 mode)
* Description    : CH549定时计数器2 T2EX引脚捕捉功能初始化（CAP2 P11）
                   UINT8 mode,边沿捕捉模式选择
                   0:T2ex从下降沿到下一个下降沿
                   1:T2ex任意边沿之间
                   3:T2ex从上升沿到下一个上升沿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP2Init(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    C_T2  = 0;
    EXEN2 = 1;
    CP_RL2 = 1;                                                                //启动T2ex的捕捉功能
    T2MOD |= mode << 2;                                                        //边沿捕捉模式选择
}
/*******************************************************************************
* Function Name  : CAP1Init(UINT8 mode)
* Description    : CH549定时计数器2 T2引脚捕捉功能初始化T2(CAP1 P10)
                   UINT8 mode,边沿捕捉模式选择
                   0:T2ex从下降沿到下一个下降沿
                   1:T2ex任意边沿之间
                   3:T2ex从上升沿到下一个上升沿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP1Init(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 1;
    C_T2 = 0;
    T2MOD = T2MOD & ~T2OE | (mode << 2) | bT2_CAP1_EN;                         //使能T2引脚捕捉功能,边沿捕捉模式选择
}
/*******************************************************************************
* Function Name  : CAP0Init(UINT8 mode)
* Description    : CH549定时计数器2 CAP0引脚捕捉功能初始化(CAP0 P36)
                   UINT8 mode,边沿捕捉模式选择
                   0:T2ex从下降沿到下一个下降沿
                   1:T2ex任意边沿之间
                   3:T2ex从上升沿到下一个上升沿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP0Init(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 1;
    C_T2 = 0;
    T2MOD |= mode << 2;                                                        //边沿捕捉模式选择
    T2CON2 = bT2_CAP0_EN;
}
#ifdef T0_INT
/*******************************************************************************
* Function Name  : mTimer0Interrupt()
* Description    : CH549定时计数器0定时计数器中断处理函数（T0强制轮询故该函数仅BUZZ模式使用）
*******************************************************************************/
void mTimer0Interrupt( void ) interrupt INT_NO_TMR0 using 1                    //timer0中断服务程序,使用寄存器组1
{
	//Systime += 3;		//32.768约等于33ms
//    mTimer_x_SetData(0, toneTimValue);					//模式1 需重新给TH0和TL0赋值
//	BUZZ_G = 0;//蜂鸣器通电
//	mDelayuS(buzzTimVol);
//	BUZZ_G = 1;//蜂鸣器断电
}
#endif
#ifdef T1_INT
/*******************************************************************************
* Function Name  : mTimer1Interrupt()
* Description    : CH549定时计数器1定时计数器中断处理函数 100us中断
*******************************************************************************/
void mTimer1Interrupt( void ) interrupt INT_NO_TMR1 using 2                //timer1中断服务程序,使用寄存器组2
{
    //方式2时，Timer1自动重装
    static UINT16 tmr1 = 0;
    tmr1++;
    if(tmr1 == 2000)                                                       //100us*2000 = 200ms
    {
        tmr1 = 0;
        SCK = ~SCK;
    }
}
#endif
/*******************************************************************************
* Function Name  : mTimer2Interrupt()
* Description    : CH549定时计数器2定时计数器中断处理函数
*******************************************************************************/
//void mTimer2Interrupt( void ) interrupt INT_NO_TMR2 using 3                //timer2中断服务程序,使用寄存器组3
//{

//}








