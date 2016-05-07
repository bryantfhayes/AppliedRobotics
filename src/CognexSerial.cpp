#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "CognexSerial.h"
#include "mraa.h"

using namespace std;

CognexSerial::CognexSerial(){
	escapeChar = '!';
}

CognexSerial::~CognexSerial(){
}

double CognexSerial::getValue(char col, int row){
	char buffer[256];
	sprintf(buffer, "gv%c%03d", col, row);
	printf("%s\n", buffer);
	writeLine(buffer);
	readLine(&responseData);
	//printf("status code: %d\nmessage: %s\n", responseData.status_code, responseData.message);
	return -1.0;
}

int CognexSerial::setOnline(bool val){
	char buffer[256];
	sprintf(buffer, "SO%d", val ? 1 : 0);
	//printf("%s\n", buffer);
	writeLine(buffer);
	EdisonSerial::readLine();
	return atoi(recvBuffer);
}

void CognexSerial::getKeypoints(double keypoints[][2]) {
  int j;
  double i;
  
  // Capture frame
  char buffer[256];
  sprintf(buffer, "SE8");
  writeLine(buffer);
  usleep(1000000);

  // Get keyvalues
  for(int n = 0; n < 2; n++){
    j = 0;
    getValue('d', 38+n);
    if(responseData.status_code == 1){
      // Analyze serial message
      string msg = string(responseData.message);
      stringstream ss(msg);
      while(ss >> i){
        keypoints[j++][n] = i;
        if(ss.peek() == ',')
          ss.ignore();
      }
    }
  }
  for(int j = 0; j < 8; j++) {
    printf("Keypoint #%d: %lf, %lf\n", j, keypoints[j][0], keypoints[j][1]);
  }
}

void CognexSerial::readLine(CognexData* data){
   char tempBuf[255];
   int idx = 0;
   int n = 0;
   int m = 0;

   while(!gameover){
       if(_uart->dataAvailable()){
            n = _uart->read(tempBuf, MAX_MSG_SIZE);
            for(int i = 0; i < n; i++){
                if(tempBuf[i] != escapeChar){
                    recvBuffer[idx] = tempBuf[i];
                    idx++;
                } else {
                    recvBuffer[idx] = '\0';
                    m = i;
                    break;
                }
            }
            data->status_code = atoi(recvBuffer);
            idx = 0;
            for(int i = m+1; i < n; i++){
                //printf("n: %d, m: %d\n", n, m);
                if(tempBuf[i] != escapeChar){
                    recvBuffer[idx] = tempBuf[i];
                    idx++;
                } else {
                    recvBuffer[idx] = '\0';
                    strcpy(data->message, recvBuffer);
                    return;
                }
            }
            
       }
   }
}


