
#include "XboxUSB.h"

#define THIS_ENDP0_SIZE 8	//端点0大小,定义在c文件以免冲突

//USB端点缓存，必须是偶地址。若为双向端点，则IN缓存一般位于OUT缓存之后64字节位置
static UINT8X Ep0Buffer[32] _at_ 0;//端点0 OUT/IN
static UINT8X Ep1Buffer[64+32] _at_ (32);//端点1 OUT&IN
static UINT8X Ep2Buffer[64+32] _at_ (128);//端点2 OUT&IN
static UINT8X Ep3Buffer[64+32] _at_ (160);//端点3 OUT&IN
static UINT8X Ep4Buffer[32] _at_ (64);//端点4 IN
//0 |32  |64 |96 |128 |192|256 |320|352
//32|32  |32 |32 |64  |64 |64  |32 |
//0 |1OUT|4IN|1IN|2OUT|2IN|3OUT|3IN|

UINT8 WakeUpEnFlag = 0;	//远程唤醒使能标志
bit Endp1Busy = 0;	//端点1传输完成控制标志位
bit Endp2Busy = 0;	//端点2传输完成控制标志位

static bit	Ready;		//USB就绪标志
static UINT8	UsbConfig;	//USB配置标志
static PUINT8	pDescr;		//描述符指针

static UINT8	SetupReq;	//Setup请求码
static UINT16	SetupLen;
static USB_SETUP_REQ	SetupReqBuf;	//Setup请求结构体，暂存Setup包


UINT8I XboxData[20];//Xbox数据


#pragma  NOAREGS


/*字符串描述符*/
static UINT8C MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };//0.语言描述符(0x0409为美式英语)
static UINT8C MyManuInfo[] = {//1.厂家信息"©Microsoft Corporation"
	46,3,194,169,77,0,105,0,99,0,114,0,111,0,115,0,111,0,102,0,116,0,32,0,
	67,0,111,0,114,0,112,0,111,0,114,0,97,0,116,0,105,0,111,0,110,0,
};
static UINT8C MyProdInfo[] = {22,3,67,0,111,0,110,0,116,0,114,0,111,0,108,0,108,0,101,0,114,0,};//2.产品信息"Controller"
//static UINT8C MySrNumInfo[] = {16,3,'0',0,'8',0,'F',0,'F',0,'C',0,'9',0,'3',0,};//3.序列号信息"08FEC93"
static UINT8C MySrNumInfo[] = {16,3,'0',0,'8',0,'F',0,'F',0,'5',0,'4',0,'9',0,};//3.序列号信息"08FE549"
static UINT8C MySafeInfo[] = {//4.安全信息"Xbox Security Method 3, Version 1.00, © 2005 Microsoft Corporation. All rights reserved."
	178,3,88,0,98,0,111,0,120,0,32,0,83,0,101,0,99,0,117,0,114,0,105,0,116,0,121,0,32,0,77,0,101,0,116,0,
	104,0,111,0,100,0,32,0,51,0,44,0,32,0,86,0,101,0,114,0,115,0,105,0,111,0,110,0,32,0,49,0,46,0,48,0,
	48,0,44,0,32,0,194,169,32,0,50,0,48,0,48,0,53,0,32,0,77,0,105,0,99,0,114,0,111,0,115,0,111,0,102,0,
	116,0,32,0,67,0,111,0,114,0,112,0,111,0,114,0,97,0,116,0,105,0,111,0,110,0,46,0,32,0,65,0,108,0,108,
	0,32,0,114,0,105,0,103,0,104,0,116,0,115,0,32,0,114,0,101,0,115,0,101,0,114,0,118,0,101,0,100,0,46,0,
};//首字节为总长度，之后固定为3以表示字符串描述符，之后为Unicode字符串

/*设备描述符*/
static UINT8C DevDesc[] = {//设备描述符
	0x12, // bLength
	0x01, // bDescriptorType
	0x00, 0x02, // bcdUSB (2.0)
	0xFF, // bDeviceClass
	0xFF, // bDeviceSubClass
	0xFF, // bDeviceProtocol
	0x08, // bMaxPacketSize0
	0x5E, 0x04, // idEVendor (Microsoft Corp.)
	0x8E, 0x02, // idProduct (Xbox360 Controller)
	0x14, 0x01, // bcdDevice
	0x01, // iManufacturer
	0x02, // iProduct
	0x03, // iSerialNumber
	0x01, // bNumConfigurations
};
/*配置描述符*/
static UINT8C CfgDesc[] = {//配置描述符
	// Configuration Descriptor
	0x09, // bLength
	0x02, // bDescriptorType (CONFIGURATION)
	0x99, 0x00, // wTotalLength (153)
	0x04, // bNumInterfaces
	0x01, // bConfigurationValue
	0x00, // iConfiguration
	0xA0, // bmAttributes
	0xFA, // bMaxPower
	/* ---------------------------------------------------- */
	// Interface 0: Control Data
	0x09, // bLength
	0x04, // bDescriptorType (INTERFACE)
	0x00, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x02, // bNumEndpoints
	0xFF, // bInterfaceClass
	0x5D, // bInterfaceSubClass
	0x01, // bInterfaceProtocol
	0x00, // iInterface
	// Unknown Descriptor (If0)
	0x11, // bLength
	0x21, // bDescriptorType
	0x00, 0x01, 0x01, 0x25, // ???
	0x81, // bEndpointAddress (IN, 1)
	0x14, // bMaxDataSize
	0x00, 0x00, 0x00, 0x00, 0x13, // ???
	0x01, // bEndpointAddress (OUT, 1)
	0x08, // bMaxDataSize
	0x00, 0x00, // ???
	// Endpoint 1: Control Surface Send
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x81, // bEndpointAddress (IN, 1)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x04, // bInterval
	// Endpoint 1: Control Surface Receive
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x01, // bEndpointAddress (OUT, 1)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x08, // bInterval
	/* ---------------------------------------------------- */
	// Interface 1: Headset (and Expansion Port?)
	0x09, // bLength
	0x04, // bDescriptorType (INTERFACE)
	0x01, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x04, // bNumEndpoints
	0xFF, // bInterfaceClass
	0x5D, // bInterfaceSubClass
	0x03, // bInterfaceProtocol
	0x00, // iInterface
	// Unknown Descriptor (If1)
	0x1B, // bLength
	0x21, // bDescriptorType
	0x00, 0x01, 0x01, 0x01, // ???
	0x82, // bEndpointAddress (IN, 2)
	0x40, // bMaxDataSize
	0x01, // ???
	0x02, // bEndpointAddress (OUT, 2)
	0x20, // bMaxDataSize
	0x16, // ???
	0x83, // bEndpointAddress (IN, 3)
	0x00, // bMaxDataSize
	0x00, 0x00, 0x00, 0x00, 0x00, 0x16, // ???
	0x03, // bEndpointAddress (OUT, 3)
	0x00, // bMaxDataSize
	0x00, 0x00, 0x00, 0x00, 0x00, // ???
	// Endpoint 2: Microphone Data Send
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x82, // bEndpointAddress (IN, 2)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x02, // bInterval
	// Endpoint 2: Headset Audio Receive
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x02, // bEndpointAddress (OUT, 2)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x04, // bInterval
	// Endpoint 3: Unknown, Send
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x83, // bEndpointAddress (IN, 3)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x40, // bInterval
	// Endpoint 3: Unknown, Receive
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x03, // bEndpointAddress (OUT, 3)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x10, // bInterval
	/* ---------------------------------------------------- */
	// Interface 2: Unknown
	0x09, // bLength
	0x04, // bDescriptorType (INTERFACE)
	0x02, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x01, // bNumEndpoints
	0xFF, // bInterfaceClass
	0x5D, // bInterfaceSubClass
	0x02, // bInterfaceProtocol
	0x00, // iInterface
	// Unknown Descriptor (If2)
	0x09, // bLength
	0x21, // bDescriptorType
	0x00, 0x01, 0x01, 0x22, // ???
	0x84, // bEndpointAddress (IN, 4)
	0x07, // bMaxDataSize
	0x00, // ???
	// Endpoint 4: Unknown, Send
	0x07, // bLength
	0x05, // bDescriptorType (ENDPOINT)
	0x84, // bEndpointAddress (IN, 4)
	0x03, // bmAttributes
	0x20, 0x00, // wMaxPacketSize
	0x10, // bInterval
	/* ---------------------------------------------------- */
	// Interface 3: Security Method
	0x09, // bLength
	0x04, // bDescriptorType (INTERFACE)
	0x03, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x00, // bNumEndpoints
	0xFF, // bInterfaceClass
	0xFD, // bInterfaceSubClass
	0x13, // bInterfaceProtocol
	0x04, // iInterface
	// Unknown Descriptor (If3)
	0x06, // bLength
	0x41, // bDescriptorType
	0x00, 0x01, 0x01, 0x03, // ???
};

/*******************************************************************************
* Function Name  : CH554USBDevWakeup()
* Description    : CH554设备模式唤醒主机，发送K信号
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554USBDevWakeup( ){
    UDEV_CTRL |= bUD_LOW_SPEED;//先变为低速
    mDelaymS(2);
    UDEV_CTRL &= ~bUD_LOW_SPEED;//再变回全速
}

/*******************************************************************************
* Function Name  : UsbXboxInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsbXboxInit(){
    IE_USB = 0;						//先失能USB中断
    USB_CTRL = 0x00;				//先设定USB设备模式
    UDEV_CTRL = bUD_PD_DIS;			//禁止DP/DM下拉电阻
	
    UDEV_CTRL &= ~bUD_LOW_SPEED;	//选择全速12M模式，默认方式
    USB_CTRL &= ~bUC_LOW_SPEED;
	
	UEP0_T_LEN = 0;					//预使用发送长度一定要清空
    UEP1_T_LEN = 0;					//预使用发送长度一定要清空
	UEP2_T_LEN = 0;					//预使用发送长度一定要清空
	UEP3_T_LEN = 0;					//预使用发送长度一定要清空
	UEP4_T_LEN = 0;					//预使用发送长度一定要清空
	
    UEP0_DMA = Ep0Buffer;										//端点0数据传输地址
    UEP1_DMA = Ep1Buffer;										//端点1数据传输地址
	UEP2_DMA = Ep2Buffer;										//端点2数据传输地址
	UEP3_DMA = Ep3Buffer;										//端点3数据传输地址
	
	UEP4_1_MOD |= bUEP1_TX_EN | bUEP1_RX_EN;					//端点1发送接收使能
    UEP4_1_MOD &= ~bUEP1_BUF_MOD;								//端点1收发各64字节缓冲区
	UEP4_1_MOD |= bUEP4_TX_EN | bUEP4_RX_EN;					//端点4发送接收使能
	
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN;					//端点2发送接收使能
	UEP2_3_MOD |= bUEP3_TX_EN | bUEP3_RX_EN;					//端点3发送接收使能
    UEP2_3_MOD &= ~bUEP2_BUF_MOD;								//端点2收发各64字节缓冲区
	UEP2_3_MOD &= ~bUEP3_BUF_MOD;								//端点3收发各64字节缓冲区
	
    USB_DEV_AD = 0x00;												//清空设备地址
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;			//启动USB设备及内部上拉及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;										//使能USB端口
    USB_INT_FG = 0xFF;												//清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;		//使能总线挂起唤醒中断、传输完成中断、总线复位中断
    IE_USB = 1;														//使能USB中断
}

/*******************************************************************************
* Function Name  : Enp1XboxIn
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1XboxIn(UINT8 *buf, UINT8 len){
	memcpy(Ep1Buffer + MAX_PACKET_SIZE, buf, len);	//Xbox模式加载上传数据
    if(Ready && !Endp1Busy){						//USB就绪且端点1空闲
        UEP1_T_LEN = len;												//设置发送长度
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		//有数据时上传数据并应答ACK
        Endp1Busy = 1;
    }
}

/*******************************************************************************
* Function Name  : Enp2XboxIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2XboxIn(UINT8 *buf, UINT8 len){
	memcpy(Ep2Buffer + MAX_PACKET_SIZE, buf, len);	//加载上传数据
    if(Ready && !Endp2Busy){						//USB就绪且端点2空闲
        UEP2_T_LEN = len;												//设置发送长度
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		//有数据时上传数据并应答ACK
        Endp2Busy = 1;
    }
}

/*******************************************************************************
* Function Name  : XboxInterrupt()
* Description    : CH549USB中断处理函数下属函数
*******************************************************************************/
void XboxInterrupt( void )				//USB中断服务程序在Xbox模式下会调用该函数
//void DeviceInterrupt( void ) interrupt INT_NO_USB using 1				//USB中断服务程序,使用寄存器组1
{
	UINT8 errflag/*, i*/;//错误标志以及公用的i
	UINT16 len;
	
    if(UIF_TRANSFER){			//USB传输完成标志
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
		case UIS_TOKEN_IN | 1:							//端点1上传
            UEP1_T_LEN = 0;														//预使用发送长度一定要清空
            UEP1_CTRL ^= bUEP_T_TOG;											//手动翻转
            Endp1Busy = 0;
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;			//默认应答NAK
            break;
		case UIS_TOKEN_OUT | 1:							//端点1下传
            if(U_TOG_OK){														//不同步的数据包将丢弃
                UEP1_CTRL ^= bUEP_R_TOG;									    //手动翻转同步标志位
				len = USB_RX_LEN;                                               //接收数据长度，数据从Ep1Buffer首地址开始存放
				//UEP1_T_LEN = len;												//设置发送长度
/**************************************************以下通信部分独立缩进**************************************************/

/**************************************************以上通信部分独立缩进**************************************************/
            }
            break;
		case UIS_TOKEN_IN | 2:							//端点2上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
            UEP2_CTRL ^= bUEP_T_TOG;                                            //手动翻转同步标志位
            Endp2Busy = 0;
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
        case UIS_TOKEN_OUT | 2:							//端点2下传
            if(U_TOG_OK){														//不同步的数据包将丢弃
                UEP2_CTRL ^= bUEP_R_TOG;									    //手动翻转同步标志位
				len = USB_RX_LEN;                                               //接收数据长度，数据从Ep2Buffer首地址开始存放
				//端点2,3,4暂不处理通信数据
            }
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
                            case 3:								//序列号信息
                                pDescr = (PUINT8)( &MySrNumInfo[0] );
                                len = sizeof( MySrNumInfo );
                                break;
							case 4:								//安全信息
                                pDescr = (PUINT8)( &MySafeInfo[0] );
                                len = sizeof( MySafeInfo );
                                break;
                            default:							//不支持的字符串描述符
                                errflag = 0xFF;
                                break;
                            }
                            break;
                        case 0x22:							//报表描述符(Xbox没有)
                            if(UsbSetupBuf->wIndexL == 0){		//接口0报表描述符
//                                pDescr = KeyRepDesc;
//                                len = sizeof(KeyRepDesc);
                            }else if(UsbSetupBuf->wIndexL == 1){//接口1报表描述符
//                                pDescr = ComRepDesc;
//                                len = sizeof(ComRepDesc);
                            }else errflag = 0xFF;				//不支持的其他接口
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
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
							case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
							case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
							case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
							case 0x03:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
							case 0x84:
                                UEP4_CTRL = UEP4_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
							case 0x04:
                                UEP4_CTRL = UEP4_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
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
								case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
								case 0x01:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点1 OUT Stall */
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
								case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
                                    break;
                                case 0x03:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
                                    break;
								case 0x84:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点4 IN STALL */
                                    break;
                                case 0x04:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点4 OUT Stall */
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


