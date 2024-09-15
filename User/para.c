
#include "para.h"

UINT8X FlashBuf[64] _at_ (2048-64);//���û�������



PUINT8X DATA_CFG = FlashBuf;//�ڴ���������Ϣָ��


void ParaSave(uint8_t pos, uint8_t num){//��������
	uint8_t i;
	uint16_t addr;
	if(pos >= 1 && pos <= 1) addr = DATA_CFG_BASE - (pos - 1) * 512;//���ô洢λ�ü���
	else return;
	
	Flash_Op_Check_Byte1 = DEF_FLASH_OP_CHECK1;//��������־��λ
	Flash_Op_Check_Byte2 = DEF_FLASH_OP_CHECK2;
	
	for(i = 0; i < num; i++){
		if(FlashErasePage(addr + i * 64)){//������ʧ��
			mDelaymS(1);
			FlashErasePage(addr + i * 64);//�ٴβ���
		}
		if(FlashProgPage(addr + i * 64, FlashBuf + i * 64, 64)){//�����ʧ��
			mDelaymS(1);
			FlashProgPage(addr + i * 64, FlashBuf + i * 64, 64);//�ٴα��
		}
	}
	
	Flash_Op_Check_Byte1 = 0;//��������־��λ
	Flash_Op_Check_Byte2 = 0;
}

void ParaLoad(){//������ȡ
//	ParaUpdate(3);
//	ParaUpdate(2);
//	ParaUpdate(1);
	memcpy(FlashBuf, (PUINT8C)DATA_CFG_BASE, 64);//��FLASH���Ƶ��ڴ�
}

void ParaUpdate(uint8_t pos){//��������
	uint16_t addr;
	uint8_t i;
	
//	if(pos >= 1 && pos <= CFG_NUM) addr = DATA_CFG_BASE - (pos - 1) * 512;//���㱾�����õ���ʼ��ַ
//	else if(pos >= 51 && pos <= 50 + CFG_NUM){
////		keyRGB(1);//����RGB��������
//		return;
//	}
//	else return;


}




















