#pragma once
#include "config.h"

void Cn2b(double yaw, double pitch, double roll, Matrix& cnb, Matrix& Q);
void Cn2b(double yaw, double pitch, double roll, double cnb[3][3], double Q[4]);