/*
 * PeakingFilter.c
 *
 *  Created on: Mar 24, 2025
 *      Author: joeln
 */


#include <stdint.h>
#include <math.h>
#include "PeakingFilter.h"


//Function for initalizing filter
void EQ_init(EQ *filt, float fs)
{
    filt->T = 1.0f/fs;

    //clear input and output
    for(uint8_t n = 0; n<3; n++)
    {
        filt->x[n] = 0.0f;
        filt->y[n] = 0.0f;
    }

    //call function to set parameters
    EQ_setParam(filt, 1.0f, 1.0f, 0.0f);
}
//T, center freq, gain, bandwidth
void EQ_setParam(EQ *filt, float fc, float g, float bw)
{
    float wc = 2.0f*tanf(M_PI*fc*filt->T);
    float Q = fc/bw;

    filt->a[0] = 4 + 2*(g/Q)*wc + (wc)*(wc);
    filt->a[1] = 2*(wc)*(wc) - 8;
    filt->a[2] = 4 - 2*(g/Q)*wc + (wc)*(wc);

    filt->b[0] = 4 + 2*(1/Q)*wc + (wc)*(wc);
    filt->b[1] = -(2*(wc)*(wc) - 8);
    filt->b[2] = -(4 - 2*(1/Q)*wc + (wc)*(wc));
}

float EQ_filter(EQ *filt, float in)
{
    //shift
    filt->x[2] = filt->x[1];
    filt->x[1] = filt->x[0];
    filt->x[0] = in;

    filt->y[2] = filt->y[1];
    filt->y[1] = filt->y[0];

    filt->y[0] = (1/filt->b[0])*((filt->a[0]*filt->x[0] + filt->a[1]*filt->x[1] + filt->a[2]*filt->x[2]) + (filt->b[1]*filt->y[1] + filt->b[2]*filt->y[2]));

    return (filt->y[0]);
}
