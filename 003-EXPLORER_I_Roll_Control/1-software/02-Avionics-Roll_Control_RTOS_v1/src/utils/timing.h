/*-----Timing_Util-----*/
#pragma once
#include <Arduino.h>

unsigned long dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;    /*Fixed working cycle interval [us]; calculating T = 1/f: 100Hz -> 10 ms = 10000 us; floating point continuously is slower and less deterministic, switching to integer math; rounded_result = (A + B/2) / B*/