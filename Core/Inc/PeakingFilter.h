/*
 * PeakingFilter.h
 *
 *  Created on: Mar 24, 2025
 *      Author: joeln
 */

#ifndef INC_PEAKINGFILTER_H_
#define INC_PEAKINGFILTER_H_

#include <stdint.h>
#include <math.h>


typedef struct{

    float T; //sample time

    float x[3]; //input signal
    float y[3]; //output signal

    //coefficiants
    float a[3];
    float b[3];
} EQ;

void EQ_init(EQ *filt, float fs);
void EQ_setParam(EQ *filt, float fc, float g, float bw);
float EQ_filter(EQ *filt, float in);


#endif /* INC_PEAKINGFILTER_H_ */
