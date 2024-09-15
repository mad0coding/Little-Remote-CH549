/*******************************************************************************
* File Name          : HidUSB.H
* Author             : L&E
* Date               : 2024/2/22
* Description        : CH549模拟CustomHID+HID复合设备
********************************************************************************/
#include "CH549.H"
#include "DEBUG.H"

#include "XboxUSB.h"
#include "para.h"

#define HID_EPIN_ADDR						0x81U//HID使用上行端点1
#define CUSTOM_HID_EPIN_ADDR				0x82U//自定义HID使用上行端点2
#define CUSTOM_HID_EPOUT_ADDR				0x02U//自定义HID使用下行端点2

#define THIS_ENDP0_SIZE         64		//端点0数据包大小
#define ENDP1_IN_SIZE           22		//HID端点IN数据包大小
#define ENDP2_IN_SIZE           64		//自定义HID端点IN数据包大小
#define ENDP2_OUT_SIZE          64		//自定义HID端点OUT数据包大小

#define USBD_MAX_NUM_INTERFACES				2//接口个数
#define USBD_HID_INTERFACE					0//HID接口编号
#define USBD_CUSTOM_HID_INTERFACE			1//自定义HID接口编号

#define CUSTOM_HID_FS_BINTERVAL		0x1//自定义HID轮询间隔
#define HID_FS_BINTERVAL			0x1//HID轮询间隔


#define Vendor_ID		0x054C	//厂商ID(Sony)
#define Product_ID		0x05C4	//产品ID(PS4)

//#define UsbSetupBuf		((PUSB_SETUP_REQ)Ep0Buffer)

#pragma  NOAREGS

extern uint8_t ifReceiving;//接收数据标志位
//extern uint8_t savePlace;//存储位置
//extern uint8_t saveGlobal;//存储中位标志

//extern UINT8 pdata WakeUpEnFlag;		//远程唤醒使能标志


void Enp1IntIn(UINT8 *buf, UINT8 len);
void Enp2IntIn(UINT8 *buf, UINT8 len);

void UsbDeviceInit();
void UsbDeviceDeinit();









