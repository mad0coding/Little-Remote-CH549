
#include "para.h"

UINT8X FlashBuf[64] _at_ (2048-64);//配置缓存数组



PUINT8X DATA_CFG = FlashBuf;//内存区配置信息指针


void ParaSave(uint8_t pos, uint8_t num){//参数保存
	uint8_t i;
	uint16_t addr;
	if(pos >= 1 && pos <= 1) addr = DATA_CFG_BASE - (pos - 1) * 512;//配置存储位置计算
	else return;
	
	Flash_Op_Check_Byte1 = DEF_FLASH_OP_CHECK1;//保护检查标志置位
	Flash_Op_Check_Byte2 = DEF_FLASH_OP_CHECK2;
	
	for(i = 0; i < num; i++){
		if(FlashErasePage(addr + i * 64)){//若擦除失败
			mDelaymS(1);
			FlashErasePage(addr + i * 64);//再次擦除
		}
		if(FlashProgPage(addr + i * 64, FlashBuf + i * 64, 64)){//若编程失败
			mDelaymS(1);
			FlashProgPage(addr + i * 64, FlashBuf + i * 64, 64);//再次编程
		}
	}
	
	Flash_Op_Check_Byte1 = 0;//保护检查标志复位
	Flash_Op_Check_Byte2 = 0;
}

void ParaLoad(){//参数读取
//	ParaUpdate(3);
//	ParaUpdate(2);
//	ParaUpdate(1);
	memcpy(FlashBuf, (PUINT8C)DATA_CFG_BASE, 64);//从FLASH复制到内存
}

void ParaUpdate(uint8_t pos){//参数更新
	uint16_t addr;
	uint8_t i;
	
//	if(pos >= 1 && pos <= CFG_NUM) addr = DATA_CFG_BASE - (pos - 1) * 512;//计算本套配置的起始地址
//	else if(pos >= 51 && pos <= 50 + CFG_NUM){
////		keyRGB(1);//键盘RGB控制清零
//		return;
//	}
//	else return;


}




















