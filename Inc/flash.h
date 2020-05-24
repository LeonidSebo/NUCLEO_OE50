#ifndef __FLASH_H
#define __FLASH_H
#include "stm32l4xx_hal.h"

#define FLASH_MEM_SIZE					FLASH_END  - FLASH_BASE + 1 // (0x40000UL)		//
#define FLASH_INIT_DATA_SIZE			3*FLASH_PAGE_SIZE
#define FLASH_BRIGHTNESS_TAB_SIZE		3*FLASH_PAGE_SIZE	//Brightness table
#define FLASH_DEVICE_INFO_SIZE			FLASH_PAGE_SIZE
#define FLASH_CONFIG_SIZE				FLASH_PAGE_SIZE
#define FLASH_HEADER_INIT_SIZE			FLASH_PAGE_SIZE

#define FLASH_INIT_BASE_ADDR			FLASH_BASE + FLASH_MEM_SIZE - FLASH_INIT_DATA_SIZE
#define FLASH_HEADER_INIT_BASE_ADDR		FLASH_INIT_BASE_ADDR
#define FLASH_INIT_DATA_BASE_ADDR		FLASH_INIT_BASE_ADDR+FLASH_HEADER_INIT_SIZE

#define FLASH_BRIGHTNESS_1_BASE_ADDR	FLASH_INIT_BASE_ADDR-FLASH_BRIGHTNESS_TAB_SIZE
#define FLASH_BRIGHTNESS_0_BASE_ADDR	FLASH_BRIGHTNESS_1_BASE_ADDR-FLASH_BRIGHTNESS_TAB_SIZE
#define FLASH_CONFIG_BASE_ADDR			FLASH_BRIGHTNESS_0_BASE_ADDR - FLASH_CONFIG_SIZE	/*0x806c000  0x801b000*/
#define FLASH_DEVICE_INFO_BASE_ADDR		FLASH_CONFIG_BASE_ADDR - FLASH_DEVICE_INFO_SIZE   	/*0x801a800*/
#define FLASH_HEADER_RECORD_SIZE		8


#define TEMP_MINMAX_SIZE				   2*sizeof(float)

#define FLASH_HEADER_RECORD_HX7816

#define CARD_ID_0		1
#define CARD_ID_1		2

//#define FLASH_HEADER_RECORD_HX7816_BOOT
typedef struct {
	uint16_t offset;
	uint16_t cmd_num;
	uint16_t size;
	uint16_t spare;
}flash_header_record_t;

typedef enum{
	FLASH_CMD_DEVICE_DESTINATION,
	FLASH_CMD_I2C_WRITE_8x8,
	FLASH_CMD_DELAY_MS,
	FLASH_CMD_DELAY_US
}flash_cmd_id_t;

typedef enum{
	FLASH_DEVICE_INFO_ID,
	FLASH_CONFIG_ID,
	FLASH_BRIGHTNESS_TABLE_0_ID,
	FLASH_BRIGHTNESS_TABLE_1_ID,
	FLASH_INIT_DATA_ID
}flash_region_id_t;

typedef struct{
	uint32_t card_ID;
	uint32_t card_revision;
}device_info_t;

typedef struct {
	uint16_t R;
	uint16_t G;
	uint16_t B;
}brightnees_t;

typedef struct {
	float temperature_min;		//degreas C
	float temperature_max;		//degreas C
	uint16_t brightnessTabNo;	//Brightness Table number: (0/1)
	uint16_t brightness;
	uint16_t bootBrightnesTabNo;;
	uint16_t bootBrightness;
	int8_t   screenOffsetHor;
	int8_t   screenOffsetVer;
	int8_t	 flipScreenHor;
	int8_t	 flipScreenVer;
}config_t;

typedef struct{
	flash_header_record_t header_record_asic_boot;
	flash_header_record_t header_record_lcos;
	flash_header_record_t header_record_vga2rgb;
	flash_header_record_t header_record_imu;
	flash_header_record_t header_record_acic_work;
}flash_init_header_t;

#endif	//__FLASH_H
