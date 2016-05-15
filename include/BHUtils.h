#ifndef __BHUtils_H
#define __BHUtils_H

#include <string.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <math.h>

#define PI 3.141592653

double interp(double n, int x1, int x2, int y1, int y2);
double rads_to_degs(double x);
double degs_to_rads(double x);
std::vector<std::string> cStringToStringVector(char* str, char delimiter);

#endif