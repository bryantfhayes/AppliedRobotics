#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "EdisonConsole.h"
#include "mraa.h"

using namespace std;

EdisonConsole::EdisonConsole(){
    fprintf(stdout, "CONSOLE:    RUNNING\n");
    running = true;
}

void EdisonConsole::readLine(void){
   fprintf(stdout, "Enter Coordinates (x,y,z):");
   cin >> recvBuffer;
}

void EdisonConsole::writeLine(char* msg, int max_msg_size){
    fprintf(stdout, "%s\n", msg);
}
