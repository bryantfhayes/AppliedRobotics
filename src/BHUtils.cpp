/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 14:38:40
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-14 18:27:00
*/

#include "BHUtils.h"

using namespace std;

double interp(double n, int x1, int x2, int y1, int y2) {
    double newRange = y2 - y1;
    double oldRange = x2 - x1;
    double newValue;

    if(oldRange == 0) {
        newValue = y1;
    } else {
        newValue = (((n - x1)*newRange) / oldRange) + y1;
    }

    return newValue;


    double slope = 1.0 * (y2 - y1) / (x2 - x1);
    int output = y1 + round(slope * (n - x1));
    if((y2 > y1) && (output > y2)) return y2;
    if((y2 > y1) && (output < y1)) return y1;
    if((y2 < y1) && (output > y1)) return y1;
    if((y2 < y1) && (output < y2)) return y2;
    return output;
}

double rads_to_degs(double x) {
    return x*(180/PI);
}

double degs_to_rads(double x) {
	return x*(PI/180);
}

vector<string> cStringToStringVector(char* str, char delimiter) {
    string s = string(str);
    vector<string> strings;
    stringstream ss(s);
    string tmp;

    while(getline(ss, tmp, delimiter)){
        strings.push_back(tmp);
    }

    return strings;
}