#ifndef __EDISONSERIAL_H
#define __EDISONSERIAL_H
#include "EdisonComm.h"
#include "mraa.hpp"

#define SERIAL_PORT "/dev/ttyMFD1"

class EdisonSerial : public EdisonComm {
    public:
        EdisonSerial();
        void readLine(void);
        void writeLine(char*,int);
    private:
        mraa::Uart* _uart;
};

#endif
