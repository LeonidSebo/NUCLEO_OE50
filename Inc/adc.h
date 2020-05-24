/*
 * adc.h
 *
 *  Created on: May 2, 2020
 *      Author: Leonid Sorkin
 */

#ifndef ADC_H_
#define ADC_H_

#define VDDA		VREFINT_CAL_VREF		//(uint32_t)*VREFINT_CAL_ADDR	//mV

#define RAW_ADC_6V_OFFSET		0
#define RAW_ADC_1V8_OFFSET		1
#define RAW_ADC_VDDA_OFFSET		2
#define RAW_ADC_TEMP_OFFSET		3
#define ADC_CONV_NUM			4

/*
 * raw VDDA thresholds =  ((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF)/voltage_mV;
 */
//#define ADC_3V3_MIN				1650					// 3V
//#define ADC_3V3_MAX				1375					// 3.6V
//#define ADC_6V_MIN				(6000 - 600)*4095/(2*3300)	// 6V - 10%		if VDDA == 3300 mV
//#define ADC_6V_MAX				(6000 + 600)*4095/(2*3300)	// 6V + 10%		if VDDA == 3300 mV
//#define ADC_1V8_MIN				(1800-180)*4095/3300	// 1.8V - 10%		if VDDA == 3300 mV
//#define ADC_1V8_MAX				(1800+180)*4095/3300	// 1.8V + 10%		if VDDA == 3300 mV


#define ADC_3V3_MIN				3000			// 3V
#define ADC_3V3_MAX				3600			// 3.6V
#define ADC_6V_MIN				(6000 - 600)/2	// 6V/2 - 10%
#define ADC_6V_MAX				(6000 + 600)/2	// 6V/2 + 10%
#define ADC_1V8_MIN				(1800-180)		// 1.8V - 10%
#define ADC_1V8_MAX				(1800+180)		// 1.8V + 10%


#define ADC_CALIBRATIION_TEMP_ADDR_LOW			*((uint32_t*)0x1FFF75A8)
#define ADC_CALIBRATIION_TEMP_ADDR_HIGHT		*((uint32_t*)0x1FFF75CA)
//#define ADC_CALIBRATIION_REF_VOLTAGE_ADDR		*((uint32_t*)0x1FFF75AA)
#define ADC_CALIBRATIION_TEMP_DEGREE_LOW		30
#define ADC_CALIBRATIION_TEMP_DEGREE_HIGHT		130
#define ADC_DELTA_CALIBRATIION_TEMP				ADC_CALIBRATIION_TEMP_DEGREE_HIGHT - ADC_CALIBRATIION_TEMP_DEGREE_LOW

#define TS_CAL1		(uint16_t)(*((uint16_t*)0x1FFF75A8))   //the temperature sensor calibration value acquired at TS_CAL1_TEMP
#define TS_CAL2		(uint16_t)(*((uint16_t*)0x1FFF75CA))    //the temperature sensor calibration value acquired at TS_CAL2_TEMP
#define TS_DELTA_CAL 		TS_CAL2-TS_CAL1

#define TS_CAL1_TEMP 	30
#define TS_CAL2_TEMP 	130

#endif /* ADC_H_ */
