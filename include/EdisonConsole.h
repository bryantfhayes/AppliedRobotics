#ifndef __EDISONCONSOLE_H
#define __EDISONCONSOLE_H
#include "EdisonComm.h"
#include "mraa.hpp"

class EdisonConsole : public EdisonComm {
    public:
        EdisonConsole();
        void readLine(void);
        void writeLine(char*);
};

#endif
