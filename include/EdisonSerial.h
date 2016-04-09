#ifndef __EDISONSERIAL_H
#define __EDISONSERIAL_H
#include "EdisonComm.h"

#define SERIAL_PORT "/dev/ttyMFD1"

class EdisonSerial : public EdisonComm{
    public:
        EdisonSerial();
        void readLine(void);
        void writeLine(void);
    private:
        mraa::Uart* _uart;
};

#endif
