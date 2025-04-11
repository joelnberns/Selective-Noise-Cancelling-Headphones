/*
 * HighPass_FirstOrder.h
 *
 *  Created on: Mar 17, 2025
 *      Author: joeln
 */

#ifndef INC_HIGHPASS_FIRSTORDER_H_
#define INC_HIGHPASS_FIRSTORDER_H_


typedef struct {
	float out;
	// previous input
	float inp;
	// filter coefficients
	float coeff;
	// sampling frequency
	float fs_Hz;

} HighPass_FirstOrder;

void HighPass_FirstOrder_Init(HighPass_FirstOrder *filt, float fc_Hz, float fs_Hz);

void HighPass_FirstOrder_SetCutoff(HighPass_FirstOrder *filt, float fc_Hz);

float HighPass_FirstOrder_Update(HighPass_FirstOrder *filt, float inp);


#endif /* INC_HIGHPASS_FIRSTORDER_H_ */
