#include "mraa.h"
#include <stdio.h>
#include <iostream>
#include "EdisonSerial.h"

using namespace std;

EdisonSerial::EdisonSerial(){
    fprintf(stdout, "Creating Edison Serial Object\n");
    try {
        _uart = new mraa::Uart(0);
    } catch (exception& e) {
        cout << e.what() << ", likely invalid platform config" << endl;
    }

    try {
        _uart = new mraa::Uart(SERIAL_PORT);
    } catch (exception& e) {
        cout << "Error while setting up raw UART, do you have a uart?" << endl;
        terminate();
    }

    if (_uart->setBaudRate(115200) != 0) {
        cout << "Error setting parity on UART" << endl;
    }

    if (_uart->setMode(8, MRAA_UART_PARITY_NONE, 1) != 0) {
        cout << "Error setting parity on UART" << endl;
    }

    if ((_uart)->setFlowcontrol(false, false) != 0) {
        cout << "Error setting flow control UART" << endl;
    }
    running = true;
}

void EdisonSerial::readLine(void){
   while(running){
        if(_uart->dataAvailable()){
            _uart->read(recvBuffer, MAX_MSG_SIZE);
            break;
        }
   }
}

void EdisonSerial::writeLine(void){

}
