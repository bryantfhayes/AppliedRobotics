#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "EdisonConsole.h"
#include "mraa.h"

using namespace std;

EdisonConsole::EdisonConsole(){
    fprintf(stdout, "CONSOLE:    RUNNING\n");
    gameover = false;
}

void EdisonConsole::readLine(void){
   fprintf(stdout, "COMMAND >");
   cin >> recvBuffer;
}

void EdisonConsole::writeLine(char* msg){
    fprintf(stdout, "%s\n", msg);
}
