#ifndef __COGNEXSERIAL_H
#define __COGNEXERIAL_H
#include "EdisonSerial.h"
#include "mraa.hpp"

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
        CognexData responseData;
};

#endif
