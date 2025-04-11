/*
 * ADC.c
 *
 *  Created on: Apr 2, 2025
 *      Author: joeln
 */

#include "ADC.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

uint8_t ADC_Init() {

	HAL_StatusTypeDef i2cStatus;

	uint8_t test;
	// reset
	HAL_Delay(100);

	//i2cStatus = ADC_Reset();


	HAL_Delay(2);

	// put device in active mode
	// 0000 0001
	i2cStatus = ADC_RegWrite(SLEEP_CFG, 0x01);

	HAL_Delay(10);



	// set I2S
	// 0100 0000
	i2cStatus = ADC_RegWrite(ASI_CFG0, 0x40);

	// set sample rate to 48 kHz, clock to fs ratio to 64
	// 0100 0100
	i2cStatus = ADC_RegWrite(ASI_STS, 0x44);

	// CLK_SRC?

	// GPIO_CFG0 GPIO ADC?

	// set GPO as general output, low
	// 0001 0011
	//i2cStatus = ADC_RegWrite(GPO_CFG0, 0x13);

	// GPI_CFG0?

	// set channel 2 to right slot 0
	// 0010 0000
	i2cStatus = ADC_RegWrite(ASI_CH2, 0x20);

	// change protocol???

	// change channel 1 input type to single ended
	// 0010 0000
	// i2cStatus = ADC_RegWrite(CH1_CFG0, 0x20);

	// change channel 2 input type to single ended
	// 0010 0000
	// i2cStatus = ADC_RegWrite(CH2_CFG0, 0x20);

	// set high pass filter to 96 Hz?
	// 0000 0010
	//i2cStatus = ADC_RegWrite(DSP_CFG0, 0x02);

	// adjust automatic gain? (P0_R112_D[7:4]), AGC_CFG0

	// voice recognition? (P0_R117_D0)

	// do these last

	// enable input channels 1 and 2
	// 1100 0000

	i2cStatus = ADC_RegWrite(IN_CH_EN, 0xC0);

	// enable output channels 1 and 2
	// 1100 0000

	i2cStatus = ADC_RegWrite(ASI_OUT_CH_EN, 0xC0);

	// power up ADC
	// 01110000

	i2cStatus = ADC_RegWrite(PWR_CFG, 0x70);



}

HAL_StatusTypeDef ADC_Reset() {
	return ADC_RegWrite(SW_RESET, 0x01);

}

HAL_StatusTypeDef ADC_RegWrite(uint8_t regAddr, uint8_t regData) {
	return HAL_I2C_Mem_Write(&ADC_I2C_HANDLE, ADC_ADDRESS << 1,  regAddr, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);

}
HAL_StatusTypeDef ADC_RegRead(uint8_t regAddr, uint8_t *regData) {
	return HAL_I2C_Mem_Read(&ADC_I2C_HANDLE, 0x9D,  regAddr, I2C_MEMADD_SIZE_8BIT, regData, 1, HAL_MAX_DELAY);
}

