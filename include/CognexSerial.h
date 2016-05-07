#ifndef __COGNEXSERIAL_H
#define __COGNEXERIAL_H
#include "EdisonSerial.h"
#include "mraa.hpp"
#include <string.h>
#include <iostream>
#include <sstream>

#define INSIGHT_SUCCESS 1

typedef struct CognexData {
	int status_code;
	char message[256];
} CognexData;

class CognexSerial : public EdisonSerial {
    public:
        CognexSerial();
        ~CognexSerial();
        double getValue(char, int);
        int setOnline(bool);
        void readLine(CognexData*);
        void getKeypoints(double keypoints[][2]);
        CognexData responseData;
};

#endif
