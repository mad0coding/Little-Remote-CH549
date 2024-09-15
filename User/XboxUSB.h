/********************************** (C) COPYRIGHT *******************************
* File Name          : XboxUSB.H
* Author             : L&E
* Version            : V1.0
* Date               : 2024/02/15
* Description        : CH549模拟Xbox控制器,支持唤醒
********************************************************************************/
#include "CH549.H"
#include "DEBUG.H"

#include "tools.h"

//#define THIS_ENDP0_SIZE			8
//#define ENDP1_IN_SIZE           32
//#define ENDP1_OUT_SIZE          32
//#define ENDP2_IN_SIZE           32
//#define ENDP2_OUT_SIZE          32
//#define ENDP3_IN_SIZE           32
//#define ENDP3_OUT_SIZE          32
//#define ENDP4_IN_SIZE           32
//#define ENDP4_OUT_SIZE          32

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

#pragma  NOAREGS

extern UINT8 WakeUpEnFlag;
extern bit Endp1Busy;	//端点1传输完成控制标志位
extern bit Endp2Busy;	//端点2传输完成控制标志位

extern UINT8I XboxData[20];

void Enp1XboxIn(UINT8 *buf, UINT8 len);
void Enp2XboxIn(UINT8 *buf, UINT8 len);

void CH554USBDevWakeup();
void UsbXboxInit();

void XboxInterrupt();



