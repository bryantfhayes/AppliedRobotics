/*
* @Author: Bryant Hayes
* @Date:   2016-05-06 23:17:23
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-23 13:44:53
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "EdisonConsole.h"
#include "mraa.h"

using namespace std;

EdisonConsole::EdisonConsole(){
	fprintf(stdout, "Initializing console...\n");
    gameover = false;
}

void EdisonConsole::readLine(void){
   fprintf(stdout, "COMMAND >");
   cin >> recvBuffer;
}

void EdisonConsole::writeLine(const char* msg){
    fprintf(stdout, "%s\n", msg);
}
