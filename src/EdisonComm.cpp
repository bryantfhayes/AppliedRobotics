/*
* @Author: Bryant Hayes
* @Date:   2016-04-14 01:22:17
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-12 23:27:12
*/

#include "EdisonComm.h"
#include "EdisonSerial.h"
#include "EdisonSocket.h"
#include "EdisonConsole.h"

EdisonComm::EdisonComm(){
	// No need for constructor of abstract class
}

EdisonComm* EdisonComm::initComm(int choice){
    switch(choice){
        case 0: 
            return new EdisonSerial;
        case 1:
            return new EdisonSocket;
        case 2:
        	return new EdisonConsole;
    }
}
