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

    fprintf(stdout, "SERIAL:     RUNNING\n");
    gameover = false;
}

void EdisonSerial::readLine(void){
   char tempBuf[255];
   int idx = 0;
   int n = 0;
   while(!gameover){
       if(_uart->dataAvailable()){
            n = _uart->read(tempBuf, MAX_MSG_SIZE);
            for(int i = 0; i < n; i++){
                if(tempBuf[i] != '\n'){
                    printf("added to buffer\n");
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

void EdisonSerial::writeLine(char* msg, int max_msg_size){
    char message[max_msg_size+1];
    sprintf(message, "%s%c", msg);
    printf("sending: %s\n", message);
    int result = _uart->write(message, max_msg_size+1);
    if(result == -1){
        perror("Problem sending serial data:");
    }

}
