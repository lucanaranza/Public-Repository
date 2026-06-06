/*Pyro module*/

// “execute command”
// the "contract"

#pragma once
#include <Arduino.h>

void pyro_init(int pin1, int pin2);
void pyro_fire(uint8_t channel, uint32_t duration_ms);