/*
 * lcos.h
 *
 *  Created on: May 6, 2020
 *      Author: Sorkin Leonid
 */

#ifndef LCOS_H_
#define LCOS_H_

#define LCOS_CTRL_REG		0

#define HX7816_INP_HSTART_ADDR			0x11
#define HX7816_INP_VSTART_ADDR			0x17
#define HX7816_PATTERN_SEL_ADDR			0x30
#define HX7816_PATTERN_VS_RESET_ADDR	0x3A
#define HX7816_PASSWORD_ADDR			0xFE

#define HX7816_INP_IF_SEL_ADDR			0xF7

#define HX7816_INP_IF_RGB				0x00
#define HX7816_INP_IF_LVDS				0x01
#define HX7816_INP_IF_PATTERN			0x02
#define HX7816_PASSWORD					0x33

#define HX7816_INP_HSTART_SIZE		2

typedef struct{
	unsigned unused		:5;
	unsigned UD			:1;
	unsigned LR			:1;
}lcosCtrlReg_t;


typedef enum{
	ASIC_RES_BYPASS,
	ASIC_RES_VGA,			//(640x480)
	ASIC_RES_SVGA, 			//(800x600)
	ASIC_RES_WSVGA,			//(1024x600)
	ASIC_RES_XGA, 			//(1024x768)
	ASIC_RES_720P, 			//(1280x720)
	ASIC_RES_WXGA,			//(1366x768)
	ASIC_RES_CUSTOM_SET		//
}asic_resolution_t;

typedef enum{
	ASIC_PATTERN_GRAY_SCASLE,
	ASIC_PATTERN_COLOR_BAR,
	ASIC_PATTERN_BORDER,
	ASIC_PATTERN_RED_COLOR,
	ASIC_PATTERN_GREEN_COLOR,
	ASIC_PATTERN_BLUE_COLOR,
	ASIC_PATTERN_WITE_COLOR,
	ASIC_PATTERN_CHESS_BOX,
	ASIC_PATTERN_RAMP_GRAY,
	ASIC_PATTERN_RAMP_RED,
	ASIC_PATTERN_RAMP_GREEN,
	ASIC_PATTERN_RAMP_BLUE,
	ASIC_PATTERN_GRILL_11,
	ASIC_PATTERN_GRILL_22,
	ASIC_PATTERN_GRILL_44,
	ASIC_PATTERN_MAX = ASIC_PATTERN_GRILL_44
}pattern_id_t;

void lcos_flip(void);
void asic_GetScreenPos(uint16_t* pBasePoz_Hor, uint16_t* pBasePoz_Ver);
void asic_ScreenOffset_Hor(void);
void asic_ScreenOffset_Ver(void);
void asic_SetPattern(uint8_t Pattern_ID,uint8_t PatternOff);

#endif /* LCOS_H_ */
