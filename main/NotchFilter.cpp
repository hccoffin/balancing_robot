#include "NotchFilter.h"
#include <math.h>

NotchFilter::NotchFilter(float w0, float ww, float freq) {
    float w0_pw = (2*freq) * tan(w0 * .5 / freq);
    float alpha = 4 + (w0_pw / freq)*(w0_pw / freq);
    float beta = 2 * ww / freq;

    c1 = alpha;
    c2 = 2*(alpha - 8);
    c3 = alpha;
    c4 = -2*(alpha - 8);
    c5 = -(alpha - beta);
    c6 = alpha + beta;

    x0 = 0;
    x1 = 0;
    x2 = 0;
    y0 = 0;
    y1 = 0;
    y2 = 0;
}

float NotchFilter::next(float val) {
    x2 = x1;
    x1 = x0;
    x0 = val;
    y2 = y1;
    y1 = y0;
    y0 = (c1*x0 + c2*x1 + c3*x2 + c4*y1 + c5*y2) / c6;
    return y0;
}