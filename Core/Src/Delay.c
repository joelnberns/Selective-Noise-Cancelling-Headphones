/*
 * Delay.c
 *
 *  Created on: Feb 14, 2025
 *      Author: joeln
 */

#include "Delay.h"
#include <stdint.h>



void Delay_Init(Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_Hz) {

	// set delay line length
	Delay_SetLength(dly, delayTime_ms, sampleRate_Hz);

	// store delay setting
	dly->mix = mix;
	dly->feedback = feedback;

	// clear delay line circular buffer, reset index
	dly->lineIndex = 0;

	for (uint32_t n = 0; n < DELAY_MAX_LINE_LENGTH; n++) {
		dly->line[n] = 0.0f;
	}

	// clear output
	dly->out = 0.0f;
}

float Delay_Update(Delay *dly, float inp){
	// get current delay line output
	float delayLineOutput = dly->line[dly->lineIndex];

	// compute current delay line input
	float delayLineInput = inp + dly->feedback*delayLineOutput;

	//store in delay line circular buffer
	dly->line[dly->lineIndex] = delayLineInput;

	// increment delay line index
	dly->lineIndex++;
	if (dly->lineIndex >= dly->lineLength) {
		dly->lineIndex = 0;
	}

	dly->out = (1.0f - dly->mix) * inp + dly->mix * delayLineOutput;

	/*
	if (dly->out > 10.0f) {
		dly->out = 1.0f;
	}
	else if (dly->out < -1.0) {
		dly->out = -1.0f;
	}
	*/

	return dly->out;

}
void Delay_SetLength(Delay *dly, float delayTime_ms, float sampleRate_Hz) {
	float testFloat = 0.001f * delayTime_ms * (float)(sampleRate_Hz);
	printf("delay time %d \n", (int32_t)(delayTime_ms));
	printf("freq %d \n", (int32_t)(sampleRate_Hz));
	dly->lineLength = (uint32_t)(0.001f * delayTime_ms * (float)(sampleRate_Hz));
	printf("line length: %d \n", (int)(dly->lineLength));
	if (dly->lineLength > DELAY_MAX_LINE_LENGTH) {
		dly->lineLength = DELAY_MAX_LINE_LENGTH;
	}
}




