#include "math.h"
double mapping(double value, double in_min, double in_max, double out_min, double out_max){

    if(value < in_min ){
        return out_min;
    } else if ( value > in_max ){
        return out_max;
    } else {
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}

