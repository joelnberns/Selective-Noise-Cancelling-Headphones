/*
 * Delay.h
 *
 *  Created on: Feb 14, 2025
 *      Author: joeln
 */

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>
#define DELAY_MAX_LINE_LENGTH 5000


typedef struct {

	float mix; // 1 = wet, 0 = dry
	float feedback; // -1 to 1, number of repeats and how much they delay

	float line[DELAY_MAX_LINE_LENGTH];
	uint32_t lineIndex;

	// Delay line length (delay time = delay line length / sample rate)
	uint32_t lineLength;

	// Output
	float out;


} Delay;

void Delay_Init(Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_Hz);
float Delay_Update(Delay *dly, float inp);
void Delay_SetLength(Delay *dly, float delayTime_ms, float sampleRate_Hz);



#endif /* DELAY_H_ */
