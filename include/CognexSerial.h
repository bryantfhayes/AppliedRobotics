#ifndef __COGNEXSERIAL_H
#define __COGNEXERIAL_H
#include "EdisonSerial.h"
#include "mraa.hpp"
#include <string.h>
#include <iostream>
#include <sstream>

#define INSIGHT_SUCCESS 1
#define CALIBRATE_STATE 6
#define RUN_STATE 7

using namespace std;

typedef struct CognexData {
	int status_code;
	char message[256];
} CognexData;

class CognexSerial : public EdisonSerial {
    public:
        CognexSerial();
        ~CognexSerial();
        double getBoardAngle();
        bool checkForDouble(string s);
        double getValue(char, int);
        int setOnline(bool);
        void readLine(CognexData*);
        void getKeypoints(double keypoints[][2]);
        void setState(int);
        void search(double* delay);
        void setKeypoint(double keypoints[][2], int idx);
        CognexData responseData;
        double unsorted_keypoints[8][2];
};

#endif
