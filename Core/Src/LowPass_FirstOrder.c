/*
 * LowPass_FirstOrder.c
 *
 *  Created on: Mar 16, 2025
 *      Author: joeln
 */

#include "LowPass_FirstOrder.h"

void LowPass_FirstOrder_Init(LowPass_FirstOrder *filt, float fc_Hz, float fs_Hz) {
	filt->fs_Hz = fs_Hz = fs_Hz;

	LowPass_FirstOrder_SetCutoff(filt, fc_Hz);

	filt->out = 0.0f;
}

void LowPass_FirstOrder_SetCutoff(LowPass_FirstOrder *filt, float fc_Hz) {
	if ( fc_Hz > (0.5f * filt->fs_Hz)) {
		fc_Hz = 0.5f * filt->fs_Hz;
	}
	else if (fc_Hz < 0.0f) {
		fc_Hz = 0.0f;
	}

	// compute and store filter coefficient
	float alpha = 6.28318530718f * fc_Hz / filt->fs_Hz; // alpha = 2 * pi * fc / fs

	filt->coeff[0] = alpha / (1.0f + alpha);
	filt->coeff[1] = 1.0f / (1.0f + alpha);
}

float LowPass_FirstOrder_Update(LowPass_FirstOrder *filt, float inp) {
	filt->out = filt->coeff[0] * inp + filt->coeff[1] * filt->out;

	if (filt->out > 1.0f) {
		filt->out = 1.0f;
	}
	else if (filt->out < -1.0f) {
		filt->out = -1.0f;
	}

	return filt->out;
}

