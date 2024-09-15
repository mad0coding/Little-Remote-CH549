
#include "HidUSB.H"

//UINT8X FlashBuf[1024] _at_ (1024);//配置缓存数组

//USB端点缓存,必须是偶地址
static UINT8X Ep0Buffer[64] _at_ 0x0000;	//端点0 OUT/IN
static UINT8X Ep1Buffer[64]  _at_ 192;		//端点1 IN
static UINT8X Ep2Buffer[64+64] _at_ 64;		//端点2 OUT&IN

static bit		Ready;
static UINT8		SetupReq;
static UINT8		UsbConfig;			//USB配置标志
static UINT16		SetupLen;
static PUINT8		pDescr;
static USB_SETUP_REQ	SetupReqBuf;		//暂存Setup包

//static UINT8 Endp1Busy = 0;			//传输完成控制标志位
//static UINT8 Endp2Busy = 0;			//传输完成控制标志位
//static UINT8 WakeUpEnFlag = 0;		//远程唤醒使能标志


#pragma  NOAREGS


/*设备描述符*/
static UINT8C DevDesc[] = {//设备描述符
	0x12,//1. 第一个字节 0x12 表示该设备描述符的长度为 18 字节。
	0x01,//2. 第二个字节 0x01 表示该描述符的类型为设备描述符 (Device Descriptor)。
	0x10,//3. 第三个字节 0x10 表示USB规范的版本号 (USB Specification Release Number) 的低字节。
	0x01,//4. 第四个字节 0x01 表示USB规范的版本号的高字节。
	0x00,//5. 第五个字节 0x00 表示该设备的设备类代码 (Device Class Code)。
	0x00,//6. 第六个字节 0x00 表示该设备的设备子类代码 (Device Subclass Code)。
	0x00,//7. 第七个字节 0x00 表示该设备的设备协议代码 (Device Protocol Code)。
	THIS_ENDP0_SIZE,//8. 第八个字节表示该设备的最大包长度 (Max Packet Size)。
	Vendor_ID & 0xFF,//9. 第九个字节表示该设备的厂商ID (Vendor ID) 的低字节。
	Vendor_ID >> 8,//10. 第十个字节表示该设备的厂商ID的高字节。
	Product_ID & 0xFF,//11. 第十一个字节表示该设备的产品ID (Product ID) 的低字节。
	Product_ID >> 8,//12. 第十二个字节表示该设备的产品ID的高字节。
	0x00,//13. 第十三个字节 0x00 表示该设备的设备版本号 (Device Release Number) 的低字节。
	0x01,//14. 第十四个字节 0x01 表示该设备的设备版本号的高字节。
	0x01,//15. 第十五个字节 0x01 表示该设备的制造商字符串描述符索引 (Manufacturer String Index)。
	0x02,//16. 第十六个字节 0x02 表示该设备的产品字符串描述符索引 (Product String Index)。
	0x00,//17. 第十七个字节 0x00 表示该设备的序列号字符串描述符索引 (Serial Number String Index)。
	0x01//18. 第十八个字节 0x01 表示该设备支持的配置数目 (Number of Configurations)。
};

/*字符串描述符*/
static UINT8C MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };//语言描述符
static UINT8C MyProdInfo[] = {28,0x03,'L',0,'i',0,'t',0,'t',0,'l',0,'e',0,'-',0,'R',0,'e',0,'m',0,'o',0,'t',0,'e',0,};//产品名称"Little-Remote"
static UINT8C MyManuInfo[] = {36,0x03,
	'L',0,'i',0,'g',0,'h',0,'t',0,'&',0,'E',0,'l',0,'e',0,'c',0,'t',0,'r',0,'i',0,'c',0,'i',0,'t',0,'y',0
};//制造者名称

/*HID类报文描述符*/
UINT8C KeyRepDesc[] = {//HID报文描述符
	//键盘
    0x05, 0x01,					//	USAGE_PAGE (Generic Desktop)
    0x09, 0x06,					//	USAGE (Keyboard)
    0xa1, 0x01,					//	COLLECTION (Application)
    0x85, 0x01,					//		REPORT_ID (1)
    0x05, 0x07,					//		USAGE_PAGE (Keyboard)
    0x19, 0xe0,					//		USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,					//		USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,					//		LOGICAL_MINIMUM (0)
    0x25, 0x01,					//		LOGICAL_MAXIMUM (1)
    0x75, 0x01,					//		REPORT_SIZE (1)
    0x95, 0x08,					//		REPORT_COUNT (8)
    0x81, 0x02,					//		INPUT (Data,Var,Abs)
    0x95, 0x01,					//		REPORT_COUNT (1)
    0x75, 0x08,					//		REPORT_SIZE (8)
    0x81, 0x03,					//		INPUT (Cnst,Var,Abs)
    0x95, 0x05,					//		REPORT_COUNT (5)
    0x75, 0x01,					//		REPORT_SIZE (1)
    0x05, 0x08,					//		USAGE_PAGE (LEDs)
    0x19, 0x01,					//		USAGE_MINIMUM (Num Lock)
    0x29, 0x05,					//		USAGE_MAXIMUM (Kana)
    0x91, 0x02,					//		OUTPUT (Data,Var,Abs)
    0x95, 0x01,					//		REPORT_COUNT (1)
    0x75, 0x03,					//		REPORT_SIZE (3)
    0x91, 0x03,					//		OUTPUT (Cnst,Var,Abs)
    0x95, 0x13,					//		REPORT_COUNT (19)按键数
    0x75, 0x08,					//		REPORT_SIZE (8)
    0x15, 0x00,					//		LOGICAL_MINIMUM (0)
    0x25, 0x65,					//		LOGICAL_MAXIMUM (101)
    0x05, 0x07,					//		USAGE_PAGE (Keyboard)
    0x19, 0x00,					//		USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,					//		USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,					//		INPUT (Data,Ary,Abs)
    0xc0,						//	END_COLLECTION
	
	//鼠标
    0x05, 0x01,					//	USAGE_PAGE (Generic Desktop)
    0x09, 0x02,					//	USAGE (Mouse)
    0xa1, 0x01,					//	COLLECTION (Application)
    0x85, 0x02,					//		REPORT_ID (2)
    0x09, 0x01,					//		USAGE (Pointer)
    0xa1, 0x00,					//		COLLECTION (Physical)
    0x05, 0x09,					//			USAGE_PAGE (Button)
    0x19, 0x01,					//			USAGE_MINIMUM (Button 1)
    0x29, 0x03,					//			USAGE_MAXIMUM (Button 3)
    0x15, 0x00,					//			LOGICAL_MINIMUM (0)
    0x25, 0x01,					//			LOGICAL_MAXIMUM (1)
    0x95, 0x03,					//			REPORT_COUNT (3)
    0x75, 0x01,					//			REPORT_SIZE (1)
    0x81, 0x02,					//			INPUT (Data,Var,Abs)
    0x95, 0x01,					//			REPORT_COUNT (1)
    0x75, 0x05,					//			REPORT_SIZE (5)
    0x81, 0x03,					//			INPUT (Cnst,Var,Abs)
    0x05, 0x01,					//			USAGE_PAGE (Generic Desktop)
    0x09, 0x30,					//			USAGE (X)
    0x09, 0x31,					//			USAGE (Y)
	0x09, 0x38,					//			USAGE (Wheel)	
    0x15, 0x81,					//			LOGICAL_MINIMUM (-127)
    0x25, 0x7f,					//			LOGICAL_MAXIMUM (127)
    0x75, 0x08,					//			REPORT_SIZE (8)
    0x95, 0x03,					//			REPORT_COUNT (3)
    0x81, 0x06,					//			INPUT (Data,Var,Rel)Rel相对值,Abs绝对值
    0xc0,						//		END_COLLECTION
    0xc0,						//	END_COLLECTION
	
	//指针位置
	0x05, 0x0d,					// USAGE_PAGE (Digitizers)
    //0x09, 0x02,					// USAGE (Pen)
	0x09, 0x04,					// USAGE (Touch Screen)
    0xa1, 0x01,					// COLLECTION (Application)
    0x85, 0x03,					//		REPORT_ID (3)
	0x09, 0x22,					//		USAGE (Finger)
    //0x09, 0x20,					//		USAGE (Stylus)
    0xa1, 0x00,					//		COLLECTION (Physical)
    0x09, 0x42,					//			USAGE (Tip Switch)
    0x09, 0x44,					//			USAGE (Barrel Switch)
    0x09, 0x3c,					//			USAGE (Invert)
    0x09, 0x45,					//			USAGE (Eraser Switch)
	0x09, 0x32,					//			USAGE (In Range)
    0x15, 0x00,					//			LOGICAL_MINIMUM (0)
    0x25, 0x01,					//			LOGICAL_MAXIMUM (1)
    0x75, 0x01,					//			REPORT_SIZE (1)
    0x95, 0x05,					//			REPORT_COUNT (5)
    0x81, 0x02,					//			INPUT (Data,Var,Abs)
    0x95, 0x01,					//			REPORT_COUNT (1)
    0x75, 0x03,					//			REPORT_SIZE (3)
    0x81, 0x03,					//			INPUT (Cnst,Var,Abs)
	0x75, 0x08,					//			REPORT_SIZE (8)
    0x09, 0x51,					//			USAGE (Contact Identifier)
    0x95, 0x01,					//			REPORT_COUNT (1)
    0x81, 0x02,					//			INPUT (Data,Var,Abs)
    0x05, 0x01,					//			USAGE_PAGE (Generic Desktop)
	0x75, 0x10,					//			REPORT_SIZE (16)
	0x26, 0xFF, 0x7F,			//			LOGICAL_MAXIMUM (32767)
	0x46, 0xFF, 0x7F,			//			PHYSICAL_MAXIMUM (32767)
    0x09, 0x30,					//			USAGE (X)
    0x09, 0x31,					//			USAGE (Y)
	0x95, 0x02,					//			REPORT_COUNT (2)
	0x81, 0x02,					//			INPUT (Data,Var,Abs)Rel相对值,Abs绝对值
    0xc0,						//		END_COLLECTION
    0xc0,						//	END_COLLECTION
	
	//媒体控制
	0x05,0x0C,					//	USAGE_PAGE (Consumer)
	0x09,0x01,					//	USAGE (Consumer Control)
	0xA1,0x01,					//	COLLECTION (Applicatior)
	0x85,0x04,					//		REPORT_ID (4)
	0xA1,0x00,					//		COLLECTION(Physical)
	0x09,0xE9,					//			USAGE (Volume Increment)
	0x09,0xEA,					//			USAGE (Volume Decrement)
	0x09,0xE2,					//			USAGE (Mute)
	0x09,0xCD,					//			USAGE (Play/Pause)
	0x09,0xB5,					//			USAGE (Scan Next Track)
	0x09,0xB6,					//			USAGE (Scan Previous Track)
	0x35,0x00,					//			PHYSICAL_MINIMUM (0)
	0x45,0x07,					//			PHYSICAL_MAXIMUM (7)
	0x15,0x00,					//			LOGICAL_MINIMUM (0)
	0x25,0x01,					//			LOCAL_MAXIMUM (1)
	0x75,0x01,					//			REPORT_SIZE (1)
	0x95,0x06,					//			REPORT_COUNT (6)
	0x81,0x02,					//			INPUT (Data,Var,Abs)
	0x75,0x01,					//			REPORT_SIZE (1)
	0x95,0x02,					//			REPORT_COUNT (2)
	0x81,0x01,					//			INPUT (Cnst,Ary,Abs)
	0xC0,						//		END_COLLECTION
	0xC0,						//	END_COLLECTION
	
	//手柄
    0x05, 0x01,					//	USAGE_PAGE (Generic Desktop)
    0x09, 0x05,					//	Usage (Game Pad)
    0xA1, 0x01,					//	Collection (Application)
    0xA1, 0x02,					//	Collection (Logical)
    0x85, 0x05,					//		Report ID (5)
    
//    0x05, 0x01,					//		Usage Page (Generic Desktop Ctrls)
    0x75, 0x08,					//		Report Size (8)
    0x95, 0x04,					//		Report Count (4)
    0x15, 0x00,					//		Logical Minimum (0)//    0x15, 0x81,         //	Logical Minimum (-127)
    0x26, 0xFF, 0x00,			//		Logical Maximum (255)//    0x25, 0x7F,         //	Logical Maximum (127)
    0x35, 0x00,					//		Physical Minimum (0)
    0x46, 0xFF, 0x00,			//		Physical Maximum (255)
    0x09, 0x30,					//		Usage (X)
    0x09, 0x31,					//		Usage (Y)
    0x09, 0x32,					//		Usage (Z)
    0x09, 0x35,					//		Usage (Rz)
    0x81, 0x02,					//		Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  
    0x75, 0x04,					//		Report Size (4)
    0x95, 0x01,					//		Report Count (1)
    0x25, 0x07,					//		Logical Maximum (7)
    0x46, 0x3B, 0x01,			//		Physical Maximum (315)
    0x65, 0x14,					//		Unit (System: English Rotation, Length: Centimeter)//0x66, 0x14, 0x00,     //	Unit (Eng Rot: Degree)
    0x09, 0x39,					//		Usage (Hat Switch)
    0x81, 0x42,					//		Input (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit)
   
    0x65, 0x00,					//		Unit (None)//0x66, 0x00, 0x00,     //	Unit (None)
    0x75, 0x01,					//		Report Size (1)
    0x95, 0x0C,					//		Report Count (12)
    0x25, 0x01,					//		Logical Maximum (1)
    0x45, 0x01,					//		Physical Maximum (1)
    0x05, 0x09,					//		Usage Page (Button)
    0x19, 0x01,					//		Usage Minimum (Button 1)
    0x29, 0x0C,					//		Usage Maximum (Button 12)
    0x81, 0x02,					//		Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x05, 0x01,					//		Usage Page (Generic Desktop Controls)
    0x09, 0x33,					//		Usage (Rx)
    0x09, 0x34,					//		Usage (Ry)
    0x26, 0xFF, 0x00,			//		Logical Maximum (255)
    0x75, 0x08,					//		Report Size (8)
    0x95, 0x02,					//		Report Count (2)
    0x81, 0x02,					//		Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  
	0xC0,						//	END_COLLECTION
	0xc0,						//	END_COLLECTION
};
UINT8C ComRepDesc[/*34*/] = {//自定义HID报文描述符
	0x06, 0x00, 0xff, 	// Usage page Vendor defined
	0x09, 0x01, 		// Local usage 1
	0xa1, 0x01, 		// Collation Application
	0x09, 0x02, 		// Local usage 2
	0x15, 0x00, 		// Logical min ( 0H )
	0x26, 0xff, 0x00,	// Logical max ( FFH )
	0x75, 0x08,  		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x81, 0x06,  		// Input ( Data, Relative, Wrap )
	0x09, 0x02, 		// Local usage 2
	0x15, 0x00,  		// Logical min ( 0H )
	0x26, 0xff, 0x00,	// Logical max ( FFH )
	0x75, 0x08, 		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x91, 0x06, 		// Output ( Data, Relative, Wrap )
	0xc0,				// END_COLLECTION
};

/*配置描述符*/
static UINT8C CfgDesc[] = {//配置描述符
	0x09,//1. 第一个字节 0x09 表示该配置描述符的长度为 9 字节。
	0x02,//2. 第二个字节 0x02 表示该描述符的类型为配置描述符 (Configuration Descriptor)。
	9+25+32,//3. 第三个字节 66 表示配置描述符的总长度 (Total Length) 的低字节。
	0x00,//4. 第四个字节 0x00 表示配置描述符的总长度的高字节。
	USBD_MAX_NUM_INTERFACES,//5. 第五个字节 0x02 表示该配置的接口数目 (NumInterfaces)。
	0x01,//6. 第六个字节 0x01 表示配置描述符的标识符 (Configuration Value)。
	0x00,//7. 第七个字节 0x00 表示该配置的描述字符串索引 (Configuration String Index)。
	0xA0,//8. 第八个字节 0xA0 表示该配置的特性标志 (Attributes)。A0=0b10100000，bit7必须1，bit6为是否自供电，bit5为是否远程唤醒
	0x32,//9. 第九个字节 0x32 表示该配置的最大功率 (Max Power)。这里的值 0x32 表示设备使用的最大电流为 (50 mA * 0x32) = 100 mA。

    0x09,0x04,USBD_HID_INTERFACE,0x00,1,0x03,0x01,0x00,0x00,//HID接口描述符,1端点
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,sizeof(KeyRepDesc)&0xFF,sizeof(KeyRepDesc)>>8,//HID类描述符
    0x07,0x05,HID_EPIN_ADDR,0x03,ENDP1_IN_SIZE,0x00,HID_FS_BINTERVAL,//端点描述符,IN端点1
	
	0x09,0x04,USBD_CUSTOM_HID_INTERFACE,0x00,2,0x03,0x00,0x00,0x00,//CustomHID接口描述符,2端点
    0x09,0x21,0x10,0x01,0x21,0x01,0X22,sizeof(ComRepDesc),0x00,//HID类描述符
    0x07,0x05,CUSTOM_HID_EPIN_ADDR,0x03,ENDP2_IN_SIZE,0x00,CUSTOM_HID_FS_BINTERVAL,//端点描述符,IN端点2
	0x07,0x05,CUSTOM_HID_EPOUT_ADDR,0x03,ENDP2_OUT_SIZE,0x00,CUSTOM_HID_FS_BINTERVAL,//端点描述符,OUT端点2
};


/*******************************************************************************
* Function Name  : CH554USBDevWakeup()
* Description    : CH554设备模式唤醒主机，发送K信号
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void CH554USBDevWakeup( ){
//    UDEV_CTRL |= bUD_LOW_SPEED;//先变为低速
//    mDelaymS(2);
//    UDEV_CTRL &= ~bUD_LOW_SPEED;//再变回全速
//}

/*******************************************************************************
* Function Name  : UsbDeviceInit()
* Description    : USB设备模式配置，设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsbDeviceInit(){
    IE_USB = 0;
    USB_CTRL = 0x00;				//先设定USB设备模式
    UDEV_CTRL = bUD_PD_DIS;			//禁止DP/DM下拉电阻
	
    UDEV_CTRL &= ~bUD_LOW_SPEED;	//选择全速12M模式，默认方式
    USB_CTRL &= ~bUC_LOW_SPEED;
	
    UEP1_T_LEN = 0;									//预使用发送长度一定要清空
	UEP2_T_LEN = 0;									//预使用发送长度一定要清空
    UEP2_DMA = Ep2Buffer;							//端点2数据传输地址
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN;		//端点2发送接收使能
    UEP2_3_MOD &= ~bUEP2_BUF_MOD;					//端点2收发各64字节缓冲区
	
    UEP0_DMA = Ep0Buffer;											//端点0数据传输地址
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);						//端点0单64字节收发缓冲区
    UEP1_DMA = Ep1Buffer;											//端点1数据传输地址
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;			//端点1发送使能 64字节缓冲区
    USB_DEV_AD = 0x00;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;			//启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;										//允许USB端口
    USB_INT_FG = 0xFF;												//清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

/*******************************************************************************
* Function Name  : UsbDeviceDeinit()
* Description    : USB关闭
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsbDeviceDeinit(){
    IE_USB = 0;			//关闭USB中断
	USB_INT_FG = 0xFF;	//清中断标志
    USB_CTRL = 0x06;	//复位USB控制寄存器并利用其复位其他寄存器
}

/*******************************************************************************
* Function Name  : Enp1IntIn
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn(UINT8 *buf, UINT8 len){
	memcpy(Ep1Buffer, buf, len);					//加载上传数据
    if(Ready && !Endp1Busy){						//USB就绪且端点1空闲
        UEP1_T_LEN = len;												//设置发送长度
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		//有数据时上传数据并应答ACK
        Endp1Busy = 1;
    }
}

/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn(UINT8 *buf, UINT8 len){
	memcpy(Ep2Buffer+MAX_PACKET_SIZE, buf, len);	//加载上传数据
    if(Ready && !Endp2Busy){						//USB就绪且端点2空闲
        UEP2_T_LEN = len;												//设置发送长度
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		//有数据时上传数据并应答ACK
        Endp2Busy = 1;
    }
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH549USB中断处理函数
*******************************************************************************/
void DeviceInterrupt( void ) interrupt INT_NO_USB using 1				//USB中断服务程序,使用寄存器组1
//void DeviceInterrupt1( void )				//USB中断服务程序,使用寄存器组1
{
	UINT8 errflag/*, i*/;//错误标志以及公用的i
    UINT16 len;
	
	if(SYSMOD == 1){	//Xbox模式
		XboxInterrupt();
		return;
	}
//	else if(SYSMOD == 3){
//		ComInterrupt();
//		return;
//	}else if(SYSMOD == 4){
//		UdiskInterrupt();
//		return;
//	}
	
    if(UIF_TRANSFER){			//USB传输完成标志
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
		case UIS_TOKEN_IN | 2:							//端点2上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
            UEP2_CTRL ^= bUEP_T_TOG;                                            //手动翻转同步标志位
            Endp2Busy = 0;
//			 UEP2_T_LEN = 2;
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
        case UIS_TOKEN_OUT | 2:							//端点2下传
            if(U_TOG_OK){														//不同步的数据包将丢弃
                UEP2_CTRL ^= bUEP_R_TOG;									    //手动翻转同步标志位
				len = USB_RX_LEN;                                               //接收数据长度，数据从Ep2Buffer首地址开始存放
				UEP2_T_LEN = len;												//设置发送长度
/**************************************************以下CustomHID通信部分独立缩进**************************************************/
#define Buf		Ep2Buffer
#define Offset	MAX_PACKET_SIZE
#define count	Buf[Offset+63]//此字节在通信中不会修改,故借用
#define packs	Buf[Offset+62]//此字节在通信中不会修改,故借用
#define place	Buf[Offset+61]//此字节在通信中不会修改,故借用
/*if(!ifReceiving){//若未在接收状态
	if((Buf[0] == 'C' && Buf[1] == 'H' || Buf[0] == 'L' && Buf[1] == 'T') && Buf[2] >= '1' && Buf[2] <= '3'){//连接指令
		if(Buf[0] == 'C') packs = 8;//键盘配置
		else packs = 4;//灯效配置
		place = Buf[2] - '0';//确定写入位置
		Buf[Offset+0] = 'R'; Buf[Offset+1] = Buf[0]; Buf[Offset+2] = Buf[1];//填入响应字节
		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;//启动上传响应主机
		count = 0;//计数置零
		ifReceiving = 1;//接收数据标志位置位
	}
//	else if(Buf[0] == 'R' && Buf[1] == 'K' && Buf[2] == 'C'){//摇杆校正指令
//		Buf[Offset+0] = 'R'; Buf[Offset+1] = 'K';//填入响应字节
//		ANA_MID_SET[0] = adcValue[0];//将当前摇杆采样值作为摇杆中位值
//		ANA_MID_SET[1] = adcValue[1];
//		Buf[Offset+2] = ANA_MID_SET[0] >> 8;//填入摇杆采样值
//		Buf[Offset+3] = ANA_MID_SET[0] & 0xFF;
//		Buf[Offset+4] = ANA_MID_SET[1] >> 8;
//		Buf[Offset+5] = ANA_MID_SET[1] & 0xFF;
//		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;//启动上传响应主机
//		saveGlobal = 1;//存储全局参数标志位置位
//	}
//	else if(Buf[0] == 'K' && Buf[1] == 'Y' && Buf[2] == 'F'){//修改按键滤波参数指令
//		Buf[Offset+0] = 'K'; Buf[Offset+1] = 'Y';//填入响应字节
//		Buf[Offset+2] = keyFltNum;//把旧参数上报
//		keyFltNum = Buf[3];//修改参数
//		Buf[Offset+3] = keyFltNum;//把新参数环回
//		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;//启动上传响应主机
//		saveGlobal = 1;//存储全局参数标志位置位
//	}
//	else if(Buf[0] == 'E' && Buf[1] == 'C' && Buf[2] == 'F'){//修改旋钮滤波参数指令
//		Buf[Offset+0] = 'E'; Buf[Offset+1] = 'C';//填入响应字节
////		TimFilterValue = Buf[3];//更新旋钮滤波参数
//		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;//启动上传响应主机
//		//saveGlobal = 1;//存储全局参数标志位置位
//	}
}
else{//数据包
	memcpy(FlashBuf + ((UINT16X)count << 6), Buf, 64);//数据包拷贝
	Buf[Offset+0] = count++;//填入序号
	Buf[Offset+1] = 'C'; Buf[Offset+2] = 'H';//填入响应字节
	UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;//启动上传响应主机
	if(count >= packs){
		count = 0;//防止越界
		if(packs == 8) savePlace = place;//确定键盘存储位置
		else savePlace = place + 50;//确定灯效存储位置
		ifReceiving = 0;//接收数据标志位复位
	}
}*/
/**************************************************以上CustomHID通信部分独立缩进**************************************************/
            }
            break;
        case UIS_TOKEN_IN | 1:							//端点1上传
            UEP1_T_LEN = 0;														//预使用发送长度一定要清空
            UEP1_CTRL ^= bUEP_T_TOG;											//手动翻转
            Endp1Busy = 0;
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;			//默认应答NAK
            break;
        case UIS_TOKEN_SETUP | 0:						//SETUP事务
            UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;	//预置NAK,防止stall之后不及时清除响应方式
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ))){
                SetupLen = ((UINT16)UsbSetupBuf->wLengthH<<8) + UsbSetupBuf->wLengthL;
                errflag = len = 0;													//默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;
                if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD){//HID类命令
                    switch( SetupReq )
                    {
                    case 0x01://GetReport
                        break;
                    case 0x02://GetIdle
                        break;
                    case 0x03://GetProtocol
                        break;
                    case 0x09://SetReport
                        break;
                    case 0x0A://SetIdle
                        break;
                    case 0x0B://SetProtocol
                        break;
                    default:							//不支持的命令或者出错
                        errflag = 0xFF;
                        break;
                    }
                }
                else{	//标准请求
                    switch(SetupReq)		//请求码
                    {
                    case USB_GET_DESCRIPTOR:		//请求获取描述符
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:								//设备描述符
                            pDescr = DevDesc;						//给出设备描述符指针
                            len = sizeof(DevDesc);					//给出设备描述符长度
                            break;
                        case 2:								//配置描述符
                            pDescr = CfgDesc;						//给出配置描述符指针
                            len = sizeof(CfgDesc);					//给出配置描述符长度
                            break;
                        case 3:								//字符串描述符
                            switch( UsbSetupBuf->wValueL )
                            {
							case 0:								//语言信息
                                pDescr = (PUINT8)( &MyLangDescr[0] );
                                len = sizeof( MyLangDescr );
                                break;
                            case 1:								//厂商信息
                                pDescr = (PUINT8)( &MyManuInfo[0] );
                                len = sizeof( MyManuInfo );
                                break;
                            case 2:								//产品信息
                                pDescr = (PUINT8)( &MyProdInfo[0] );
                                len = sizeof( MyProdInfo );
                                break;
                            default:							//不支持的字符串描述符
                                errflag = 0xFF;
                                break;
                            }
                            break;
                        case 0x22:							//报表描述符
                            if(UsbSetupBuf->wIndexL == 0){		//接口0报表描述符
                                pDescr = KeyRepDesc;
                                len = sizeof(KeyRepDesc);
                            }
							else if(UsbSetupBuf->wIndexL == 1){	//接口1报表描述符
                                pDescr = ComRepDesc;
                                len = sizeof(ComRepDesc);
                            }
                            else errflag = 0xFF;				//不支持的其他接口
                            break;
                        default:							//不支持的命令或者出错
                            errflag = 0xFF;
                            break;
                        }
                        if(SetupLen > len){
                            SetupLen = len;	//限制总长度
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                        //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:			//请求设置USB设备地址
                        SetupLen = UsbSetupBuf->wValueL;	//暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if(SetupLen >= 1) len = 1;
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        if(UsbConfig){
#ifdef DE_PRINTF
                            printf("SET CONFIG.\n");
#endif
                            Ready = 1;	//SetConfig命令一般代表USB枚举完成的标志
                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:			//Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )//端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
							case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:		//其他不支持的端点
                                errflag = 0xFF;
                                break;
                            }
                        }
                        if((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE){//设备
                            WakeUpEnFlag &= ~0x01;
//                            printf("Wake up\n");
                            break;
                        }
                        else errflag = 0xFF;                                                //不是端点不支持
                        break;
                    case USB_SET_FEATURE:                                               /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )              /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 ){
                                    WakeUpEnFlag |= 0x01;                                   /* 设置唤醒使能标志 */
//                                    printf("Enable Remote Wakeup.\n");
                                }
                                else errflag = 0xFF;                                        /* 操作失败 */
                            }
                            else errflag = 0xFF;                                            /* 操作失败 */
                        }
                        else if( (UsbSetupBuf->bRequestType & 0x1F) == 0x02 )        /* 设置端点 */
                        {
                            if( ( ( (UINT16)UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( (UINT16)UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    errflag = 0xFF;					//操作失败
                                    break;
                                }
                            }
                            else errflag = 0xFF;					//操作失败
                        }
                        else errflag = 0xFF;						//操作失败
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 ) len = 2;
                        else len = SetupLen;
                        break;
                    default:
                        errflag = 0xFF;								//操作失败
                        break;
                    }
                }
            }
            else{
                errflag = 0xFF;										//包长度错误
            }
            if(errflag == 0xFF){
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len){                                                //上传数据或者状态阶段返回0长度包
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else{
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
			case UIS_TOKEN_IN | 0:							//端点0上传
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;    //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                            //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:					//端点0下传
            len = USB_RX_LEN;
            if(SetupReq == 0x09){
//				printf("%d	%d	",Ep0Buffer[0],Ep0Buffer[1]);
                if(Ep0Buffer[0]){
//                    printf("Light on Num Lock LED!\n");
                }
                else if(Ep0Buffer[0] == 0){
//                    printf("Light off Num Lock LED!\n");
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG;							//同步标志位翻转
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;										//写0清空中断
    }
    else if(UIF_BUS_RST){		//设备模式USB总线复位中断
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = UEP_T_RES_NAK;
//        UEP2_CTRL = UEP_T_RES_NAK;
		UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        Ready = 0;
        Endp1Busy = 0;
        Endp2Busy = 0;
        WakeUpEnFlag = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    else if(UIF_SUSPEND){		//USB总线挂起/唤醒完成
        UIF_SUSPEND = 0;
        if( USB_MIS_ST & bUMS_SUSPEND ){		//挂起
            WakeUpEnFlag |= 0x02;
#ifdef DE_PRINTF
            printf( "z" );						//睡眠状态
//             while ( XBUS_AUX & bUART0_TX )
//             {
//                 ;    //等待发送完成
//             }
#endif
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                              //USB或者RXD0有信号时可被唤醒
//             PCON |= PD;                                                          //睡眠
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = 0x00;
        }
        else{									//唤醒
            WakeUpEnFlag &= ~0x02;
#ifdef DE_PRINTF
            printf( "w" );
#endif
        }
    }
    else{										//意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;						//清中断标志
#ifdef DE_PRINTF
        printf("UnknownInt  \n");
#endif
    }
}









