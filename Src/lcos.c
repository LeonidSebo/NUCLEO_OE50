/*
 * lcos.c
 *
 *  Created on: May 6, 2020
 *      Author: Leonid Sorkin
 */
#include "main.h"
extern config_t* pConfigStruct;
extern uint8_t sw_error;
extern I2C_HandleTypeDef hi2c1;
extern irq_flg_t  irq_flg;

device_state_t lcos_state = {0};
device_state_t asic_state = {0};

int16_t asic_BasePos_Hor;
int16_t asic_BasePos_Ver;

void lcos_flip(void ){
	uint8_t ctrlReg = (pConfigStruct->flipScreenHor<<7)+(pConfigStruct->flipScreenVer<<6)+0x21;
	sw_error = HAL_I2C_Mem_Write(&hi2c1, LCOS_I2C_ADDR<<1, LCOS_CTRL_REG, sizeof(uint8_t), &ctrlReg, sizeof(ctrlReg), I2C_TX_TIMEOUT);
	if(sw_error == HAL_OK){
		lcos_state.i2c_error = 0;
	} else{
		lcos_state.i2c_error = 1;
//		irq_flg.irq_LCOS = 1;
	}
}

void asic_ScreenOffset_Hor(void){
	int16_t ScrPos = asic_BasePos_Hor + pConfigStruct->screenOffsetHor;
	sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_HSTART_ADDR, sizeof(uint8_t), (uint8_t*)&ScrPos, sizeof(ScrPos), I2C_TX_TIMEOUT);
	if(sw_error == HAL_OK){
		asic_state.i2c_error = 0;
	} else{
		asic_state.i2c_error = 1;
	}
}
void asic_ScreenOffset_Ver(void){
	int16_t ScrPos = asic_BasePos_Ver + pConfigStruct->screenOffsetVer;
	sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_VSTART_ADDR, sizeof(uint8_t), (uint8_t*)&ScrPos, sizeof(ScrPos), I2C_TX_TIMEOUT);
	if(sw_error == HAL_OK){
		asic_state.i2c_error = 0;
	} else{
		asic_state.i2c_error = 1;
	}
}

void asic_GetScreenPos(uint16_t* pBasePoz_Hor, uint16_t* pBasePoz_Ver){
	sw_error = HAL_I2C_Mem_Read(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_HSTART_ADDR, sizeof(uint8_t),(uint8_t*)&asic_BasePos_Hor, sizeof(asic_BasePos_Hor), I2C_RX_TIMEOUT);
	if(sw_error == HAL_OK){
		asic_state.i2c_error = 0;
		sw_error = HAL_I2C_Mem_Read(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_VSTART_ADDR, sizeof(uint8_t), (uint8_t*)&asic_BasePos_Ver, sizeof(asic_BasePos_Ver), I2C_RX_TIMEOUT);
	}
	if(sw_error != HAL_OK){
		asic_state.i2c_error = 1;
		irq_flg.irq_ASIC = 1;
	}
}

void asic_SetPattern(uint8_t Pattern_ID,uint8_t PatternOff){
	uint8_t data[2];
	if(PatternOff){
		data[0] = HX7816_INP_IF_RGB;
		sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_IF_SEL_ADDR, sizeof(uint8_t), data, 1, I2C_TX_TIMEOUT);
	}else if(Pattern_ID <= ASIC_PATTERN_MAX){
		data[0] = HX7816_INP_IF_PATTERN;
		sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_INP_IF_SEL_ADDR, sizeof(uint8_t), data, 1, I2C_TX_TIMEOUT);
		if(sw_error == HAL_OK){
			asic_state.i2c_error = 0;
			data[0] = 0xFF;
			data[1] = 0xFF;
			data[2] = 0x7F;
			sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_PATTERN_VS_RESET_ADDR, sizeof(uint8_t), data, 3, I2C_TX_TIMEOUT);
			data[0] = ASIC_RES_XGA | Pattern_ID;
			sw_error = HAL_I2C_Mem_Write(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_PATTERN_SEL_ADDR, sizeof(uint8_t), data, 1, I2C_TX_TIMEOUT);
		}
		if(sw_error != HAL_OK) {
			asic_state.i2c_error = 1;
			irq_flg.irq_ASIC = 1;
		}
	}else{
		sw_error = PARAM_ERROR;
	}

}
