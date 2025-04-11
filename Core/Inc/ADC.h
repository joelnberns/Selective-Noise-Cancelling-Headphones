/*
 * ADC.h
 *
 *  Created on: Apr 2, 2025
 *      Author: joeln
 */


#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define ADC_I2C_HANDLE	hi2c3
#define ADC_ADDRESS 0x4E

#define PAGE_CFG	0x00
#define SW_RESET	0x01
#define SLEEP_CFG	0x02
#define SHDN_CFG	0x05
#define ASI_CFG0	0x07
#define ASI_CFG1	0x08
#define ASI_CFG2	0x09
#define ASI_MIX_CFG	0x0A
#define ASI_CH1		0x0B
#define ASI_CH2		0x0C
#define ASI_CH3		0x0D
#define	ASI_CH4		0x0E
#define MST_CFG0	0x13
#define MST_CFG1	0x14

#define ASI_STS		0x15
#define GPO_CFG0	0x22

#define CH1_CFG0	0x3C
#define CH2_CFG0	0x41
#define DSP_CFG0	0x6B
#define IN_CH_EN 	0x73
#define ASI_OUT_CH_EN	0x74
#define PWR_CFG		0x75

extern uint8_t ADC_CONFIG_SETTINGS8[];

// I2C handle
extern I2C_HandleTypeDef ADC_I2C_HANDLE;

// functions
uint8_t ADC_Init();
HAL_StatusTypeDef ADC_Reset();

HAL_StatusTypeDef ADC_RegWrite(uint8_t regAddr, uint8_t regData);
HAL_StatusTypeDef ADC_RegRead(uint8_t regAddr, uint8_t *regData);

#endif /* INC_ADC_H_ */
