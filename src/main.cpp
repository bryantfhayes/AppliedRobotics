/*
* @Author: Bryant Hayes
* @Date:   2016-05-12 23:20:03
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-17 22:01:58
*/

//TODO: Add comments to functions
//TODO: Make a list of prototypes
//TODO: Make camera communication redundant
//TODO: Add logging and change printf's to EdisonComm class method
//TODO: Make sure camera orientation lines up with arm calibration @mechanical
//TODO: Get version 1 of the app working

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

//TODO: Counting increases even when Bad Servo Value achieved



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
    idle, calibrate_arm, calibrate_cam, custom, fish_random, fish_smart
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
state_t curr_state = idle;
state_t last_state = curr_state;
double keypoints[8][2];

//
// Idle Function
//
void idle_state_func() {
    DEBUG("Entering idle state\n");
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
    robot->setPosition(0, 39, -21);
    while(curr_state == calibrate_arm && !gameover){}
    return;
}

//
// Calibrate the camera
//
void calibrate_cam_state_func() {
    DEBUG("Entering calibrate cam state\n");
    DEBUG("In-Sight 7000 Calibrating...\n");
    cognexCom->setState(CALIBRATE_STATE);
    cognexCom->setState(RUN_STATE);
    cognexCom->getKeypoints(keypoints);
    keypointsReceived = true;
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

//
// Fish using information form camera
//
void fish_smart_state_func() {
    DEBUG("Entering fish smart state\n");
    robot->enable(true);
    int upVal = -19;
    int idx = 0;
    int numOfAttempts = 2;
    double delay = 0.0;
    while(curr_state == fish_smart && !gameover) {
        robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
        usleep(1000000);
        cognexCom->setKeypoint(keypoints, idx);
        for(int attempt = 0; attempt < numOfAttempts; attempt++){
            if(curr_state != fish_smart) return;
            robot->setPosition(keypoints[idx][0], keypoints[idx][1], upVal);
            usleep((rand() % 3 + 2) * 1500000);
            cognexCom->search(&delay);
            usleep((delay*1000));
            robot->grab();
            robot->toss();
        }
        idx++;
        if(idx >= 8) {
            idx = 0;
        }
    }
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
            robot->toss();
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
                              fish_smart_state_func};
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
        if(com != NULL) 
            com->gameover = true;
        gameover = true;
    }
}

//
// Print the various commands system accepts
//
void usage() {
    //TODO: Add usage/help menu
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
    }  else if(cmd == "shake") {
        robot->shake();
    } 

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
  return 0;
}

//
// Responsible for setting up all system interrupts.
//
int setupInterrupts() {
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);
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
    if (com != NULL) {
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
