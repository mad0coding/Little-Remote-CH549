#ifndef _PARA_H_
#define _PARA_H_

#include "CH549.H"
#include "DEBUG.H"

#include "FlashRom.H"


//��ǰ60k(0x0000~0xEFFF)ΪCodeFlash
//����1k(0xF000~0xF3FF)ΪDataFlash
//���3k(0xF400~0xFFFF)ΪBootLoader��ConfigInfo
#define DATA_CFG_BASE		0xF200//����������ʼ
//#define DATA_GLOB_BASE		0xE800//ȫ��������ʼ
//#define DATA_LIGHT_BASE		0xE700//��Ч������ʼ
//��������ռCodeFlash�ĺ�1k��DataFlash��1k,ÿ��512B,�ռ�4��,Ŀǰʹ�ú�3��,�Ӻ���ǰ��
//ȫ������ռCodeFlash�ĵ�����2k�Ŀ�ͷ����,Ŀǰʹ��64B
//��Ч����ռCodeFlash�ĵ�����3k,ÿ��256B,�ռ�4��,Ŀǰʹ�ú�3��,�Ӻ���ǰ��
//Ŀǰȫ���洢����ʹ��4k�ռ�,ʣ�����ռ�Ϊ57k

//#define CFG_NUM		3//���ø���

#define CFG_THIS		(DATA_CFG)
#define CFG_HEAD_0			(*(CFG_THIS + 0))
#define CFG_HEAD_1			(*(CFG_THIS + 1))
#define CFG_HEAD_2			(*(CFG_THIS + 2))
#define CFG_HEAD_3			(*(CFG_THIS + 3))

#define CFG_RF_DR			(*(CFG_THIS + 4))
#define CFG_RF_PWR			(*(CFG_THIS + 5))
#define CFG_RF_CH			(*(CFG_THIS + 6))
#define CFG_RF_RSV1			(*(CFG_THIS + 7))
#define CFG_RF_ADDR(n)		(*(CFG_THIS + 8 + (n)))
#define CFG_RF_RSV2			(*(CFG_THIS + 13))

#define CFG_RFA_T			(*(CFG_THIS + 14))
#define CFG_RFA_WAIT		(*(CFG_THIS + 15))
#define CFG_RFA_RSV1		(*(CFG_THIS + 16))
#define CFG_RFA_RSV2		(*(CFG_THIS + 17))

#define CFG_PWR_WAIT0		(*(CFG_THIS + 18))
#define CFG_PWR_WAIT1		(*(CFG_THIS + 19))
#define CFG_PWR_OFF			(*(CFG_THIS + 20))
#define CFG_PWR_RSV			(*(CFG_THIS + 21))

#define CFG_RK_MID(n)		(*((PUINT16X)(CFG_THIS + 22 + (n) * 2)))
#define CFG_RK_MAP(n)		(*(CFG_THIS + 30 + (n)))
#define CFG_RK_DIR(n)		(*(CFG_THIS + 34))


#define CFG_ACS(a)			(*((PUINT8C)(a) + 0))
#define CFG_K_ID(a)			(*((PUINT8C)(a) + 0))
#define CFG_K_MODE(a)		(*((PUINT8C)(a) + 1))
#define CFG_K_KEY(a)		(*((PUINT8C)(a) + 2))
#define CFG_K_FUNC(a)		(*((PUINT8C)(a) + 3))
#define CFG_K_X(a)			(*((PUINT16C)((a) + 2)))
#define CFG_K_Y(a)			(*((PUINT16C)((a) + 4)))
#define CFG_K_T(a)			(*((PUINT16C)((a) + 4)))
#define CFG_RGB_R			(*(CFG_THIS + 476))
#define CFG_RGB_G			(*(CFG_THIS + 477))
#define CFG_RGB_B			(*(CFG_THIS + 478))
#define CFG_KB_DIR			(*(CFG_THIS + 479))
#define CFG_SCN_W			(*((PUINT16C)(CFG_THIS + 480)))
#define CFG_SCN_H			(*((PUINT16C)(CFG_THIS + 482)))
#define CFG_R_MODE			(*(CFG_THIS + 484))
#define CFG_R_KEY(n)		(*(CFG_THIS + 485 + (n)))
#define CFG_R_FUNC			(*(CFG_THIS + 490))
#define CFG_E_MODE(i)		(*(CFG_THIS + 496 + (i) * 6))
#define CFG_E_KEY(i,n)		(*(CFG_THIS + 497 + (i) * 6 + (n)))
#define CFG_E_FUNC(i)		(*(CFG_THIS + 500 + (i) * 6))
#define CFG_E_DIR(i)		(*(CFG_THIS + 501 + (i) * 6))
#define CFG_ALL_PRI			(*(CFG_THIS + 511))

#define CFGb_R_DIRx			((CFG_R_DIR >> 0) & 1)
#define CFGb_R_DIRy			((CFG_R_DIR >> 1) & 1)
#define CFGb_R_DIRr			((CFG_R_DIR >> 2) & 1)
#define CFGb_Rk_MODE		(CFG_R_MODE >> 4)
#define CFGb_R_MODE			(CFG_R_MODE & 0x0F)
#define CFGb_Ek_MODE(i)		(CFG_E_MODE(i) >> 4)
#define CFGb_E_MODE(i)		(CFG_E_MODE(i) & 0x0F)
#define CFGb_RGB_COLORFUL	(CFG_RGB_CYCLE >> 4)
#define CFGb_RGB_WAVE		(CFG_RGB_CYCLE & 0x0F)
#define CFGb_RGB_RK			((CFG_RGB_SET >> 7) & 1)
#define CFGb_RGB_CLICKER	((CFG_RGB_SET >> 6) & 1)
#define CFGb_RGB_LOOP		((CFG_RGB_SET >> 5) & 1)
#define CFGb_RGB_TIME		(CFG_RGB_SET & 0x0F)



extern UINT8X FlashBuf[64];//���û�������

extern PUINT8C DATA_CFG;//������������Ϣָ��


void ParaSave(uint8_t pos, uint8_t num);//��������
void ParaLoad();//������ȡ
void ParaUpdate(uint8_t pos);//��������



#endif



