
#include "tools.h"


#pragma  NOAREGS


uint16_t AdcValue[5] = {2048,2048,2048,2048,2550};

//uint8_t NRF_state = 0;//0为已断连,1为断连预备,2为正常
//uint8_t NRF_Rx_pkg[32];
uint8_t NRF_KEY[24];
uint8_t NRF_EC;
int16_t NRF_RK[4];

int16_t IO_RK[4];
//uint8_t IO_read[24];//读取IO输入
//uint8_t IO_quies[24];//IO静默计数
uint8_t IO_use[16];//使用IO输入

int16_t Center_LRXY[4] = {Center_LX,Center_LY,Center_RX,Center_RY};//摇杆中位校正


uint8_t STD_pkg[16] = {0x55,1/*01*/,0x00,0x00,0x00,0x00,
						0,0,0,0,0,0,0,0,
						0,0xAA};//标准数据包(ID改为10,部分留空)
//0x55,ID,key1-8,kl+kr+sw1-2+key0,empty,empty,				6/6
//ADC1_H,ADC1_L,ADC2_H,ADC2_L,ADC3_H,ADC3_L,ADC4_H,ADC4_L,	8/14
//~SUM(from 10 to ADC4_L),0xAA								2/16



//uint8_t NRF_Unpack(uint8_t CheckSum)
//{
//	int8_t i = 0;
//	uint8_t sum = 0;
//	uint32_t key_unpacking = 0;
//	if(NRF_Rx_pkg[0] != 0x55) return 0x55;//帧头错误
//	if(NRF_Rx_pkg[15] != 0xAA) return 0xAA;//帧尾错误
//	if(NRF_Rx_pkg[1] != 0x01) return 0x01;//帧类型错误
//	if(CheckSum){//若需要检查和校验
//		for(i = 1; i < 14; i++){
//			sum += NRF_Rx_pkg[i];
//		}
//		if(sum != (uint8_t)~NRF_Rx_pkg[14]) return 0xFF;//和校验错误
//	}
//	key_unpacking = (NRF_Rx_pkg[2] << 16) | (NRF_Rx_pkg[3] << 8) | NRF_Rx_pkg[4];
//	for(i = 23; i >= 0; i--){//i=23访问的是NRF_Rx_pkg[2]的最高位
//		NRF_KEY[23 - i] = /*!*/((key_unpacking >> i) & 0x01);
//	}
//	NRF_EC = NRF_Rx_pkg[5];
//	NRF_RK[0] = (short)((NRF_Rx_pkg[6] << 8) | NRF_Rx_pkg[7]) >> 4;
//	NRF_RK[1] = (short)((NRF_Rx_pkg[8] << 8) | NRF_Rx_pkg[9]) >> 4;
//	NRF_RK[2] = (short)((NRF_Rx_pkg[10] << 8) | NRF_Rx_pkg[11]) >> 4;
//	NRF_RK[3] = (short)((NRF_Rx_pkg[12] << 8) | NRF_Rx_pkg[13]) >> 4;
//	return 0;
//}

//0x55,0x01,key1-8,key0+kl+kr+sw1-2,empty,empty,			6/6
void IO_read(void){
	uint8_t i;
	int32_t io_adc_tmp;
	for(i = 0; i < 4; i++){
		io_adc_tmp = (short)AdcValue[i] - Center_LRXY[i];
		if(io_adc_tmp > 0) io_adc_tmp = io_adc_tmp * 32767 / (4096 - Center_LRXY[i]);
		else if(io_adc_tmp < 0) io_adc_tmp = io_adc_tmp * 32767 / Center_LRXY[i];
		IO_RK[i] = (short)io_adc_tmp;
		
		//if(ABS(IO_RK[i]) > 260*16) active |= (1<<i);
	}
	IO_use[0] = !KP_1;	IO_use[1] = !KP_2;	IO_use[2] = !KP_3;	IO_use[3] = !KP_4;
	IO_use[4] = !KP_5;	IO_use[5] = !KP_6;	IO_use[6] = !KP_7;	IO_use[7] = !KP_8;
	IO_use[8] = !KP_L;	IO_use[9] = !KP_R;
	IO_use[10] = !SW_1;	IO_use[11] = !SW_2;
	IO_use[12] = !KP_0;
}


uint8_t IO_pack(void){
	uint8_t i;
	uint16_t rk_pos = 0;
	uint16_t compress = 0;//压缩:12按键放入16位数中

	for(i = 0; i < 13; i++){
		if(IO_use[i]) compress |= (1<<(15-i));
	}
	
	STD_pkg[2] = (compress >> 8) & 0xFF;
	STD_pkg[3] = (compress >> 0) & 0xFF;
	STD_pkg[4] = 0;
	STD_pkg[5] = 0;
	
	rk_pos = IO_RK[0];
	STD_pkg[6] = (rk_pos >> 8) & 0xFF;
	STD_pkg[7] = rk_pos & 0xFF;
	rk_pos = IO_RK[1];
	STD_pkg[8] = (rk_pos >> 8) & 0xFF;
	STD_pkg[9] = rk_pos & 0xFF;
	rk_pos = IO_RK[2];
	STD_pkg[10] = (rk_pos >> 8) & 0xFF;
	STD_pkg[11] = rk_pos & 0xFF;
	rk_pos = IO_RK[3];
	STD_pkg[12] = (rk_pos >> 8) & 0xFF;
	STD_pkg[13] = rk_pos & 0xFF;
	
	STD_pkg[14] = STD_pkg[1];
	for(i = 2; i < 14; i++){
		STD_pkg[14] += STD_pkg[i];//计算和校验
	}
	STD_pkg[14] = ~STD_pkg[14];//取反
	
	return compress;
}

uint8_t ActiveCheck(void){//活动检查
	uint8_t i, ans = 0;
	for(i = 0; i < 11; i++){//除开关和电源键之外的按键
		if(i == 8) continue;
		if(IO_use[i]) ans++;
	}
	for(i = 0; i < 4; i++){//摇杆
		if(IO_RK[i] < -10000 || IO_RK[i] > 10000) ans++;
	}
	return ans;
}










//END





