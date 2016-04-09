#ifndef __EDISONCOMM_H
#define __EDISONCOMM_H

#include <stdio.h>
#include <stdlib.h>
#include "mraa.hpp"
#define MAX_MSG_SIZE 2048


class EdisonComm{
    public:
        EdisonComm();
        bool running;
        char recvBuffer[MAX_MSG_SIZE];
        virtual void readLine(void)  = 0;
        virtual void writeLine(void) = 0;
        static EdisonComm* initComm(int);
};


#endif
