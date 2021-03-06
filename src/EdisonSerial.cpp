/*
* @Author: Bryant Hayes
* @Date:   2016-05-07 17:14:25
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-23 13:45:00
*/

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

    if (_uart->setBaudRate(9600) != 0) {
        cout << "Error setting baudrate on UART" << endl;
    }

    if (_uart->setMode(8, MRAA_UART_PARITY_NONE, 1) != 0) {
        cout << "Error setting parity on UART" << endl;
    }

    if ((_uart)->setFlowcontrol(false, false) != 0) {
        cout << "Error setting flow control UART" << endl;
    }

    fprintf(stdout, "Initializing serial connection...\n");
    gameover = false;

    escapeChar = '\r';

    _uart->setTimeout(5,0,0);
    _uart->flush();
    flushInput();
}

void EdisonSerial::readLine(void){
   char tempBuf[255];
   int idx = 0;
   int n = 0;
   while(!gameover){
       if(_uart->dataAvailable()){
            n = _uart->read(tempBuf, MAX_MSG_SIZE);
            //printf("%s\n", tempBuf);
            for(int i = 0; i < n; i++){
                if(tempBuf[i] != escapeChar){
                    recvBuffer[idx] = tempBuf[i];
                    idx++;
                } else {
                    recvBuffer[idx] = '\0';
                    return;
                }
            }
       }
   }
}

void EdisonSerial::writeLine(const char* msg){
    char message[MAX_MSG_SIZE];
    sprintf(message, "%s\r", msg);
    //printf("sending: %s with message length %d\n", message, strlen(message));
    int result = _uart->writeStr(message);
    if(result == -1){
        perror("Problem sending serial data:");
    }

}

void EdisonSerial::flushInput(void) {
    char tmpBuf[256];
    while(_uart->dataAvailable()) {
        _uart->read(tmpBuf, 1);
    }
}
