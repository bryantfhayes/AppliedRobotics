#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "EdisonSerial.h"
#include "mraa.h"

using namespace std;

EdisonSerial::EdisonSerial(){
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

    fprintf(stdout, "SERIAL:     RUNNING\n");
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

void EdisonSerial::writeLine(char* msg, int max_msg_size){
    int result = _uart->write(msg, max_msg_size);
    if(result == -1){
        perror("Problem sending serial data:");
    }

}
