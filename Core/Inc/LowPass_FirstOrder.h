/*
 * LowPass_FirstOrder.h
 *
 *  Created on: Mar 16, 2025
 *      Author: joeln
 */

#ifndef INC_LOWPASS_FIRSTORDER_H_
#define INC_LOWPASS_FIRSTORDER_H_

typedef struct {
	float out;
	// filter coefficients
	float coeff[2];
	// sampling frequency
	float fs_Hz;

} LowPass_FirstOrder;

void LowPass_FirstOrder_Init(LowPass_FirstOrder *filt, float fc_Hz, float fs_Hz);

void LowPass_FirstOrder_SetCutoff(LowPass_FirstOrder *filt, float fc_Hz);

float LowPass_FirstOrder_Update(LowPass_FirstOrder *filt, float inp);



#endif /* INC_LOWPASS_FIRSTORDER_H_ */
