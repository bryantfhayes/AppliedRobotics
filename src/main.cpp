/*
* @Author: Bryant Hayes
* @Date:   2016-05-12 23:20:03
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-29 23:01:11
*/

//TODO: Add comments to functions
//TODO: Make a list of prototypes
//TODO: Make camera communication redundant
//TODO: Add logging and change printf's to EdisonComm class method
//TODO: Make sure camera orientation lines up with arm calibration @mechanical
//TODO: Get version 1 of the app working
//TODO: Counting increases even when Bad Servo Value achieved

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <pthread.h>
#include <cstdlib>
#include <string.h>
#include <time.h>

#include "mraa.hpp"
#include "EdisonComm.h"
#include "CognexSerial.h"
#include "BHUtils.h"
#include "FishingRobot.h"



#define TRIGGER_PIN 12

// MACRO FUNCTIONS
#define DEBUG(x) do { std::cerr << x; } while (0)
inline int max ( int a, int b ) { return a > b ? a : b; }

//
// Prototypes
//
void idle_state_func();
void calibrate_arm_state_func();
void calibrate_cam_state_func();
void custom_state_func();
void fish_smart_state_func();
void fish_random_state_func();
void ready_state_func();
void loadCalibration(double keypoints[][2]);
void saveCalibration(double keypoints[][2]);
void* updateState(void* data);
void sig_handler(int signo);
void usage();
void getCommand();
int setupStateControl();
int setupEnvironment(int argc, char* argv[]);
int setupComs();
int setupInterrupts();
void cleanExit();

using namespace std;

enum state_t {
    idle, calibrate_arm, calibrate_cam, custom, fish_random, fish_smart, ready
};

enum com_mode_t {
    serial, wireless, console, test
};

//
// GLOBAL VARIABLES
//
FishingRobot* robot;
pthread_t state_control_thread;
com_mode_t mode = console; //DEFAULT
bool gameover = false;
bool keypointsReceived = false;
EdisonComm* com;
CognexSerial* cognexCom;
static state_t curr_state = idle;
static state_t last_state = curr_state;
double keypoints[8][2];
mraa::Gpio* triggerPin;
mraa::Gpio* fishPin;
static bool lookForFish = false;
static int fishCount = 0;
bool keypointsEnabledArr[8] = {true, true, true, true, true, true, true, false};


void fishInterrupt(void* args) {
    //DEBUG("fish isr triggered\n");
    if (curr_state == fish_smart && lookForFish) {
        fishCount++;
    }
}

void triggerLowInterrupt(void* args) {
    DEBUG("ISR Triggered\n");
    if(curr_state == fish_smart) {
        DEBUG("Disabling fish_smart mode\n");
        curr_state = ready;
        cognexCom->gameover = true;
    }
}

//
// Idle Function
//
void idle_state_func() {
    DEBUG("Entering idle state\n");
    robot->setPosition(-15,25,-15,2500000);
    robot->enable(false);
    while(curr_state == idle && !gameover){}
    return;
}

//
// Calibrate the arm 
//
void calibrate_arm_state_func() {
    DEBUG("Entering calibrate arm state\n");
    robot->enable(true);
    robot->setPosition(0, 39, -22);
    while(curr_state == calibrate_arm && !gameover){}
    return;
}

//
// Calibrate the camera
//
void calibrate_cam_state_func() {
    DEBUG("Entering calibrate cam state\n");
    DEBUG("In-Sight 7000 Calibrating...\n");
    char buffer[255];
    cognexCom->setState(CALIBRATE_STATE);
    cognexCom->setState(RUN_STATE);
    cognexCom->getKeypoints(keypoints);
    robot->boardAngle = cognexCom->getBoardAngle();
    if (keypoints[0][0] == 0.0 || keypoints[0][1] == 0.0) {
        com->writeLine("error");
        keypointsReceived = false;
    } else {
        sprintf(buffer, "%lf", robot->boardAngle);
        com->writeLine(buffer);
        keypointsReceived = true;
    }
    curr_state = last_state;
    return;
}

//
// Freemove
//
void custom_state_func() {
    DEBUG("Entering custom state\n");
    while(curr_state == custom && !gameover) {}
}

void ready_state_func() {
    robot->enable(true);
    robot->setPosition(-13, 22, -16);
    while(curr_state == ready && !gameover) {
        if (triggerPin->read() == 0) {
            fprintf(stdout,"Switching to GAME MODE\n");
            curr_state = fish_smart;
        }
    }
}

//
// Fish using information form camera
//
void fish_smart_state_func() {
    DEBUG("Entering fish smart state\n");
    robot->enable(true);
    int upVal = -21;
    int idx = 1;
    int numOfAttempts = 1;
    double delay = 0.0;
    char buffer[255];
    double extra_delay[8] = {0.0,0.0,0.0,0.0,25000.0,0.0,0.0,0.0};

    com->writeLine("state:fish_smart");

    while(curr_state == fish_smart && !gameover) {
        robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
        //usleep(1000000);
        cognexCom->setKeypoint(keypoints, idx);
        for(int attempt = 0; attempt < numOfAttempts; attempt++){
            // Send keypoint update
            sprintf(buffer, "info:Attempting to fish keypoint #%d\n", idx);
            com->writeLine(buffer);
            // Send speed update
            sprintf(buffer, "speed:%lf", delay);
            com->writeLine(buffer);
            if(curr_state != fish_smart) break;
            robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
            usleep(3000000);
            cognexCom->search(&delay);
            if(curr_state != fish_smart) break;
            // Flush buffer if error in response
            if ((delay*1000 - 50000 + extra_delay[idx]) > 1){
                usleep((delay*1000 - 50000 + extra_delay[idx]));
            } else {
                cognexCom->flushInput();
            }
            if(curr_state != fish_smart) break;
            robot->grab(idx);
            lookForFish = true;
            robot->setFishPosition();
            usleep(500000);
            if(fishCount >= 2) {
                fprintf(stdout, "fish detected!\n");
                robot->shake();
            } else {
                fprintf(stdout, "no fish detected!\n");
            }
            lookForFish = false;
            fishCount = 0;
        }
        while (((++idx > 7) || !keypointsEnabledArr[idx]) && curr_state == fish_smart) {
            if(idx >= 8) {
                idx = 0;
            }
        }
    }

    com->writeLine("state:ready");

    // Clear input buffer and resume operation
    cognexCom->flushInput();
    cognexCom->gameover = false;
}

//
// Fish using random guessing
//
void fish_random_state_func() {
    DEBUG("Entering fish random state\n");
    robot->enable(true);
    int idx = 0;
    int numOfAttempts = 2;
    int interval = 1;
    int upVal = -17;
    while(curr_state == fish_random && !gameover) {
        robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
        usleep(2000000);
        for(int attempt = 0; attempt < numOfAttempts; attempt++){
            robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
            //interval = rand() % 3 + 1; // 3 to 7 seconds
            usleep(interval * 1000000);
            robot->grab();
            robot->setFishPosition();
            robot->shake();
        }
        idx++;
        if(idx >= 8) {
            idx = 0;
        }
    }
}

//
// Responsible for loading calibrated key points from a file
// rather than asking the camera. Mainly for testing purposes.
//
void loadCalibration(double keypoints[][2]) {
    char buffer[256];
    FILE *fp = fopen("include/calibration.txt", "r");
    if(fp != NULL){
        for(int j = 0; j < 8; j++){
            fgets(buffer, 255, (FILE*)fp);
            string line = string(buffer);
            vector<double> pts;
            stringstream ss(line);
            double i;
            while(ss >> i){
                pts.push_back(i);
                if(ss.peek() == ',')
                    ss.ignore();
            }
            if(pts.size() == 2){
                keypoints[j][0] = pts.at(0);
                keypoints[j][1] = pts.at(1);
            }
            printf("Loaded keypoint %d: %lf, %lf\n", j, keypoints[j][0], keypoints[j][1]);
        }
        keypointsReceived = true;
        fclose(fp);
    } else {
        printf("Problem loading file\n");
    }
}

//
// Responsible for saving calibrated key points to a file
//
void saveCalibration(double keypoints[][2]) {
    char buffer[256];
    if(keypointsReceived){
        FILE *fp = fopen("include/calibration.txt", "w");
        for(int j = 0; j < 8; j++){
            fprintf(fp, "%lf,%lf\n", keypoints[j][0], keypoints[j][1]);
        }
        fclose(fp);
        DEBUG("Keypoints saved successfully!\n");
    } else {
        fprintf(stderr, "Cannot write because keypoints are not known!\n");
    }
}

//
// Monitor state and update internal state machine to drive gameplay
//
void* updateState(void* data) {
    void (* state[])(void) = {idle_state_func, 
                              calibrate_arm_state_func, 
                              calibrate_cam_state_func, 
                              custom_state_func, 
                              fish_random_state_func, 
                              fish_smart_state_func,
                              ready_state_func};
    void (* fn)(void);
    while(!gameover) {
        fn = state[curr_state];
        fn();    
    }
    pthread_exit(NULL);
}

//
// Responsible for handling CTRL-C interrupt.
//
void sig_handler(int signo) {
    if (signo == SIGINT) {
        DEBUG("[SHUT DOWN]\n");
        if(com != NULL) {
            com->gameover = true;
        }
        if(cognexCom != NULL){
            cognexCom->gameover = true;
        }
        gameover = true;
    }
}

//
// Print the various commands system accepts
//
void usage() {
    //TODO: Add usage/help menu
}

void acknowledge(string cmd) {
    char buffer[32];
    sprintf(buffer, "%s,ok",cmd.c_str());
    com->writeLine(buffer);
}

//
// Responsible for receiving and parsing input from remote system
// and then saving servo_values in memory.
//
void getCommand(){
    // Command String
    string cmd;

    // Blocking until message is received
    com->readLine();

    // Parse Command
    vector<string> input = cStringToStringVector(com->recvBuffer, ',');
    if (input.size() >= 1) {
        cmd = input.at(0);
    } else {
        return;
    }
    cout << cmd;
    // Perform Action
    if (cmd == "xyz") {
        if (input.size() == 4) {
            robot->setPosition(stod(input.at(1), NULL), stod(input.at(2), NULL), stod(input.at(3), NULL));
        } else {
            return;
        }
    } else if(cmd == "calibrate_arm") {
        last_state = curr_state;
        curr_state = calibrate_arm;
    } else if(cmd == "calibrate_cam") {
        last_state = curr_state;
        curr_state = calibrate_cam;
    } else if (cmd == "idle"){
        curr_state = idle;
    } else if (cmd == "check_fish") {
        robot->setFishPosition();
    } else if(cmd == "custom") {
        last_state = curr_state;
        curr_state = custom;
    } else if(cmd == "fish_random") {
        last_state = curr_state;
        curr_state = fish_random;
    } else if(cmd == "fish_smart") {
        last_state = curr_state;
        curr_state = fish_smart;
    } else if(cmd == "kp") {
        if (input.size() == 2 && keypointsReceived) {
            robot->setPosition(keypoints[stoi(input.at(1), NULL)][0], keypoints[stoi(input.at(1), NULL)][1], -19);
        } else if (keypointsReceived) {
            fprintf(stderr, "ERROR: Incorrect number of arguements\n");
            return;
        } else {
            fprintf(stderr, "ERROR: No Valid Keypoints\n");
            return;
        }
    } else if(cmd == "set_speed") {
        if (input.size() == 2) {
            robot->setSpeed(stoi(input.at(1), NULL));
        } else {
            fprintf(stderr, "ERROR: Incorrect number of arguments\n");
            return;
        }
    } else if(cmd == "toss") {
        robot->toss();
    } else if(cmd == "grab") {
        robot->grab(); 
    } else if(cmd == "load_calibration") {
        loadCalibration(keypoints);
    } else if(cmd == "save_calibration") {
        saveCalibration(keypoints);
    } else if(cmd == "help") {
        usage();
    } else if (cmd == "has_fish") {
        fprintf(stdout, "has fish?: %s\n", fishCount >= 2 ? "YES" : "NO");
    } else if(cmd == "shake") {
        robot->shake();
    } else if (cmd == "hello") {
        DEBUG("Connection received\n");
        fprintf(stdout, "Set Online Return Value: %d\n", cognexCom->setOnline(true));
    } else if (cmd == "ready") {
        last_state = curr_state;
        curr_state = ready;
    } else if (cmd == "test_keypoints") {
        if (keypointsReceived) {
            for (int i = 0; i < 8; i++) {
                robot->setPosition(keypoints[i][0], keypoints[i][1], -20, 6000000);
            }
        }
        
    } else if (cmd == "custom_keypoints") {
        fprintf(stdout, "New keypoints enabled array = {");
        for (int i = 1; i < input.size(); i++) {
            int val = stoi(input.at(i));
            keypointsEnabledArr[i-1] = val;
            fprintf(stdout, "%d,", val);
        }
        fprintf(stdout, "}\n");
    } else {
        return;
    }

    acknowledge(cmd);

    return;
}

//---------------------------------- SETUP ----------------------------------//

//
// Responsible for setting up the multiple threads active in program
//
int setupStateControl() {
    int retVal = pthread_create(&state_control_thread, NULL, updateState, NULL);
    if(retVal){
        fprintf(stderr, "Error: Unable to create state control thread\n");
        return 1;
    }
}

//
// Responsible for parsing command line arguements and determining
// which mode to run in.
//
int setupEnvironment(int argc, char* argv[]){
    // Example for reference: http://www.gnu.org/software/libc/manual/html_node/Example-of-Getopt.html#Example-of-Getopt
    int index;
    int c;

    opterr = 0;
    while ((c = getopt (argc, argv, "wt")) != -1){
        switch (c){
          case 'w':
            mode = wireless;
            break;
          case 't':
            mode = test;
            break;
          case '?':
            if (isprint (optopt))
              fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
              fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
            return 1;
          default:
            break;
        }
    }

    return 0;
}

//
// Responsible for initializing which ever COM mode user decides to use.
//
int setupComs(){
  com = EdisonComm::initComm((int)mode);
  cognexCom = new CognexSerial;
  if(com == NULL || cognexCom == NULL) {
    return 1;
  } else {
    return 0;
  }
}

//
// Responsible for setting up all system interrupts.
//
int setupInterrupts() {
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);
    // Also setup trigger pin
    triggerPin = new mraa::Gpio(TRIGGER_PIN);
    if (triggerPin == NULL) {
        return 1;
    }
    int response = triggerPin->dir(mraa::DIR_IN);
    if (response != MRAA_SUCCESS) {
        return 1;
    }
    response = triggerPin->mode(mraa::MODE_PULLUP);
    if (response != MRAA_SUCCESS) {
        return 1;
    }
    // Setup ISR trigger on rising edge
    triggerPin->isr(mraa::EDGE_RISING, &triggerLowInterrupt, NULL);
    
    printf("pin = %s\n", triggerPin->read() ? "HIGH" : "LOW");

    fishPin = new mraa::Gpio(11);
    if (fishPin == NULL) {
        return 1;
    }
    response = fishPin->dir(mraa::DIR_IN);
    if (response != MRAA_SUCCESS) {
        return 1;
    }
    response = fishPin->mode(mraa::MODE_PULLUP);
    if (response != MRAA_SUCCESS) {
        return 1;
    }
    // Setup ISR trigger on falling edge
    fishPin->isr(mraa::EDGE_FALLING, &fishInterrupt, NULL);

    return 0;
}

//
// Terminate all open threads and delete all created objects
//
void cleanExit() {
    int rPtr;
    if (&state_control_thread != NULL) {
        fprintf(stdout, "Terminating state control thread...\n");
        pthread_join(state_control_thread, (void**)&rPtr);
    }  
    if (com != NULL && mode != wireless) {
        delete com; 
    }
    if (cognexCom != NULL) {
        delete cognexCom;
    }
    if (robot != NULL) {
        delete robot;
    }
    fprintf(stdout, "Exiting Safely...\n");
}

//
// Main 
//
int main(int argc, char* argv[]) { 
    // Variables
    int err = 0;

    // Initialize robot
    robot = new FishingRobot;

    // Setup
    err += setupInterrupts();
    err += setupEnvironment(argc, argv);
    err += setupComs();
    err += setupStateControl();

    if(err > 0) {
        fprintf(stderr, "ERROR: Setup Failed");
        cleanExit();
    }

    // Main Loop
    while(!gameover){
        getCommand();
    }

    cleanExit();

    return MRAA_SUCCESS;
}
