#pragma once

#include <cstdint>

void InitRNG(unsigned long seed);
uint32_t RandUint32();
double RandDouble();
double RandFromRange(double a, double b);
