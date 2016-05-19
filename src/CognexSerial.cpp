/*
* @Author: Bryant Hayes
* @Date:   2016-05-12 23:20:18
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-18 22:31:44
*/

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
	writeLine(buffer);
	readLine(&responseData);
	//printf("status code: %d\nmessage: %s\n", responseData.status_code, responseData.message);
  if (responseData.status_code == 1 && checkForDouble(string(responseData.message))) {
    return stod(responseData.message, NULL);
  } else {
    return -1;
  }
}

bool CognexSerial::checkForDouble(string s) {
  istringstream ss(s);
  double d;
  return (ss >> d) && (ss >> std::ws).eof();
}

int CognexSerial::setOnline(bool val){
	char buffer[256];
	sprintf(buffer, "SO%d", val ? 1 : 0);
	//printf("%s\n", buffer);
	writeLine(buffer);
	EdisonSerial::readLine();
	return atoi(recvBuffer);
}

double CognexSerial::getBoardAngle() {
  return getValue('G', 21);
}

void sortKeypoints(double keypoints[][2]) {
  double smallest = 999.9;
  int smallest_idx = 0;
  double tmp[2];
  for(int j = 0; j < 8; j++){
    smallest_idx = j;
    smallest = keypoints[j][1];
    for(int k = j+1; k < 8; k++){
      if(keypoints[k][1] <= smallest){
        smallest = keypoints[k][1];
        smallest_idx = k;
      }
    }
    // SWAP
    tmp[0] = keypoints[j][0];
    tmp[1] = keypoints[j][1];
    keypoints[j][0] = keypoints[smallest_idx][0];
    keypoints[j][1] = keypoints[smallest_idx][1];
    keypoints[smallest_idx][0] = tmp[0];
    keypoints[smallest_idx][1] = tmp[1];
  }
}

void CognexSerial::getKeypoints(double keypoints[][2]) {
  int j;
  double i;
  
  //RESET KEYPOINTS
  for(int j = 0; j < 8; j++) {
    keypoints[j][0] = 0.0;
    keypoints[j][1] = 0.0;
  }

  // Capture frame
  char buffer[256];
  sprintf(buffer, "SE8");
  writeLine(buffer);
  usleep(1000000);

  // Get keyvalues
  for(int n = 0; n < 2; n++){
    j = 0;
    getValue('d', 53+n);
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

  for(int i = 0; i < 8; i++){
    unsorted_keypoints[i][0] = keypoints[i][0];
    unsorted_keypoints[i][1] = keypoints[i][1];
  }
  sortKeypoints(keypoints);

  printf("CALIBRATED NEW KEYPOINTS\n");
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

void CognexSerial::setState(int state) {
  char buffer[256];
  printf("Changing to state %d\n", state);
  switch (state) {
    case CALIBRATE_STATE:
      sprintf(buffer, "SI%c%03d%d", 'N', RUN_STATE, 0);
      writeLine(buffer);
      usleep(10000);
      sprintf(buffer, "SI%c%03d%d", 'N', CALIBRATE_STATE, 1);
      writeLine(buffer);
      usleep(4000000);
      flushInput();
      break;
    case RUN_STATE:
      sprintf(buffer, "SI%c%03d%d", 'N', RUN_STATE, 1);
      writeLine(buffer);
      usleep(10000);
      sprintf(buffer, "SI%c%03d%d", 'N', CALIBRATE_STATE, 0);
      writeLine(buffer);
      usleep(4000000);
      flushInput();
      break;  
  }
  
}

void CognexSerial::search(double* delay){
  char buffer[256];
  sprintf(buffer, "SI%c%03d%d", 'N', 8, 1);
  printf("sending %s\n", buffer);
  writeLine(buffer);
  usleep(100000);
  flushInput();

  EdisonSerial::readLine();
  *delay = atof(recvBuffer);
  printf("Fish in %lf milliseconds!\n", *delay);

  sprintf(buffer, "SI%c%03d%d", 'N', 8, 0);
  writeLine(buffer);
  usleep(20000);
  flushInput();

  return;
}
void CognexSerial::setKeypoint(double keypoints[][2], int idx){
  char buffer[256];
  for(int i = 0; i < 8; i++){
    if(keypoints[idx][0] == unsorted_keypoints[i][0] && keypoints[idx][1] == unsorted_keypoints[i][1]){
      idx = i;
      break;
    }
  }

  sprintf(buffer, "SI%c%03d%d", 'N', 9, idx);
  writeLine(buffer);
  usleep(10000);
  flushInput();
}


