#include "EdisonComm.h"
#include "EdisonSerial.h"
#include "EdisonSocket.h"

EdisonComm::EdisonComm(){

}

EdisonComm* EdisonComm::initComm(int choice){
    switch(choice){
        case 0: 
            return new EdisonSerial;
        case 1:
            return new EdisonSocket;
    }
}
