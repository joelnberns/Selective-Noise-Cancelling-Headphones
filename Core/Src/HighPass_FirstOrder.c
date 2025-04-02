/*
 * HighPass_FirstOrder.c
 *
 *  Created on: Mar 17, 2025
 *      Author: joeln
 */

#include "HighPass_FirstOrder.h"

void HighPass_FirstOrder_Init(HighPass_FirstOrder *filt, float fc_Hz, float fs_Hz) {
	filt->fs_Hz = fs_Hz = fs_Hz;

	HighPass_FirstOrder_SetCutoff(filt, fc_Hz);

	filt->out = 0.0f;
	filt->inp = 0.0f;
}

void HighPass_FirstOrder_SetCutoff(HighPass_FirstOrder *filt, float fc_Hz) {
	if ( fc_Hz > (0.5f * filt->fs_Hz)) {
		fc_Hz = 0.5f * filt->fs_Hz;
	}
	else if (fc_Hz < 0.0f) {
		fc_Hz = 0.0f;
	}

	// compute and store filter coefficient
	float alpha = 6.28318530718f * fc_Hz / filt->fs_Hz; // alpha = 2 * pi * fc / fs

	filt->coeff = 1.0f / (1.0f + alpha);
}

float HighPass_FirstOrder_Update(HighPass_FirstOrder *filt, float inp) {
	filt->out = filt->coeff * (inp - filt->inp + filt->out);

	if (filt->out > 1.0f) {
		filt->out = 1.0f;
	}
	else if (filt->out < -1.0f) {
		filt->out = -1.0f;
	}

	return filt->out;
}

