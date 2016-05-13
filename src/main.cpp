//
// File: main.cpp
// Author: Bryant Hayes
// Description: Low level code for controlling robotic arm based on receieved serial commands.
//

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <pthread.h>
#include <cstdlib>
#include "mraa.hpp"
#include <string.h>
#include "EdisonComm.h"
#include "CognexSerial.h"
#include <time.h>
#include "pca9685.h"
#include "IKHelper.h"

//
// DEFINES
//
#define TRUE 1
#define FALSE 0
#define SERIAL_PORT "/dev/ttyMFD1"
#define SERVO_PERIOD 20000
#define DEFAULT_SCALED_PWM 1245
#define MSG_SIZE 32
#define SERIAL_MODE 0
#define WIRELESS_MODE 1
#define CONSOLE_MODE 2
#define TEST_MODE 3

// Servo safety bounds
#define SERVO_X_MIN 830
#define SERVO_X_MAX 1770
#define SERVO_Y_MIN 730
#define SERVO_Y_MAX 1395
#define SERVO_Z_MIN 730
#define SERVO_Z_MAX 1620


//
// Prototypes
//
void updateServos(upm::PCA9685*, int[]);


inline int max ( int a, int b ) { return a > b ? a : b; }

using namespace std;

// Struct for passing information to pthread
typedef struct servo_thread_struct {
    upm::PCA9685 *servos;
    int* servo_values;
} servo_thread_struct;

typedef struct location_t {
    int x;
    int y;
    int z;
} location_t;

enum state_t {
    idle, calibrate_arm, calibrate_cam, custom, fish_random, fish_smart
};

//
// GLOBAL VARIABLES
//
static bool running = false;
static bool gameover = false;
static bool keypointsReceived = false;
static time_t last_interrupt_time = 0;
static EdisonComm* edisonCom;
static CognexSerial* cognexCom;
pthread_mutex_t my_lock;
mraa::Aio* a0;
mraa::Gpio* servoEnablePin;
state_t curr_state = idle;
state_t last_state = idle;
double keypoints[8][2];
int armSpeed = 50;


void idle_state_func(void* data) {
    running = false;
    while(curr_state == idle && !gameover){}
    return;
}

void calibrate_arm_state_func(void* data) {
    int* servo_values = ((servo_thread_struct*)data)->servo_values;
    running = true;
    XYZ_to_PWM(1.5,39,-21,servo_values);
    while(curr_state == calibrate_arm && !gameover){
    }
    return;
}

void calibrate_cam_state_func(void* data) {
    printf("In-Sight 7000 Entering Calibration Mode...\n");
    cognexCom->setState(CALIBRATE_STATE);
    cognexCom->setState(RUN_STATE);
    printf("In-Sight 7000 Calibration Complete!\n");
    int* servo_values = ((servo_thread_struct*)data)->servo_values;
    cognexCom->getKeypoints(keypoints);
    keypointsReceived = true;
    curr_state = last_state;
    return;
}

void tossFish(upm::PCA9685* servos, int* servo_values) {
    int lastSpeed = armSpeed;
    armSpeed = 100;
    XYZ_to_PWM(-30,20,-10,servo_values);
    usleep(500000);
    XYZ_to_PWM(0,39,-23,servo_values);
    usleep(1500000);
    armSpeed = lastSpeed;
    XYZ_to_PWM(0,39,-20.5,servo_values);
    usleep(1000000);
}

void grabFish(upm::PCA9685* servos, int* servo_values, int keypoint_idx) {
    int lastSpeed = armSpeed;
    armSpeed = 99;
    XYZ_to_PWM(keypoints[keypoint_idx][0], keypoints[keypoint_idx][1], -24,  servo_values);
    usleep(1500000-armSpeed*10000);
    armSpeed = lastSpeed;
    usleep(300000);
    XYZ_to_PWM(0,39,-17,servo_values);
    usleep(1000000);
}

void custom_state_func(void* data) {
    running = true;
    while(curr_state == custom && !gameover) {}
}

void fish_smart_state_func(void* data) {
    running = true;
    // Values to fling fish off
    upm::PCA9685* servos = ((servo_thread_struct*)data)->servos;
    int* servo_values = ((servo_thread_struct*)data)->servo_values;
    int keypoint_idx = 0;
    int attempt = 0;
    int upVal = -19;
    int downVal = -24.5;
    double delay;
    while(curr_state == fish_smart && !gameover) {
        XYZ_to_PWM(keypoints[keypoint_idx][0], keypoints[keypoint_idx][1], upVal, servo_values);
        usleep(1000000);
        cognexCom->setKeypoint(keypoints, keypoint_idx);
        for(attempt = 0; attempt < 1; attempt++){
            if(curr_state != fish_smart) return;
            XYZ_to_PWM(keypoints[keypoint_idx][0], keypoints[keypoint_idx][1], upVal, servo_values);
            usleep((rand() % 3 + 2) * 1500000);
            cognexCom->search(&delay);
            usleep((delay*1000));
            grabFish(servos, servo_values, keypoint_idx);
            tossFish(servos, servo_values);
            //usleep(3000000);
        }
        keypoint_idx++;
        if(keypoint_idx >= 8) {
            keypoint_idx = 0;
        }
    }
}

void fish_random_state_func(void* data) {
    running = true;
    int keypoint_idx = 1;
    int attempt = 0;
    int upVal = -20.8;
    int interval = 1;
    upm::PCA9685* servos = ((servo_thread_struct*)data)->servos;
    int* servo_values = ((servo_thread_struct*)data)->servo_values;

    while(curr_state == fish_random && !gameover) {
        XYZ_to_PWM(keypoints[keypoint_idx][0], keypoints[keypoint_idx][1], upVal, servo_values);
        usleep(2000000);
        for(attempt = 0; attempt < 1; attempt++){
            XYZ_to_PWM(keypoints[keypoint_idx][0], keypoints[keypoint_idx][1], upVal, servo_values);
            //interval = rand() % 3 + 1; // 3 to 7 seconds
            usleep(interval * 1000000);
            grabFish(servos, servo_values, keypoint_idx);
            tossFish(servos, servo_values);
        }
        keypoint_idx++;
        if(keypoint_idx >= 8) {
            keypoint_idx = 1;
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
        printf("Keypoitns saved successfully!\n");
    } else {
        printf("Cannot write because keypoints are not known!\n");
    }
}

//
// Monitor state and update internal state machine to drive gameplay
//
void* updateState(void* data){
    void (* state[])(void*) = {idle_state_func, calibrate_arm_state_func, calibrate_cam_state_func, custom_state_func, fish_random_state_func, fish_smart_state_func};
    void (* fn)(void*);
    while(!gameover) {
        fn = state[curr_state];
        fn(data);    
    }
    pthread_exit(NULL);
}

//
// Responsible for setting PWM duty cycle to control servo position.
//
void updateServos(upm::PCA9685* servos, int servo_values[]){
    // Safety range check
    if (((servo_values[0] > SERVO_X_MAX) || (servo_values[0] < SERVO_X_MIN)) || ((servo_values[1] > SERVO_Y_MAX) || (servo_values[1] < SERVO_Y_MIN)) || ((servo_values[2] > SERVO_Z_MAX) || (servo_values[2] < SERVO_Z_MIN))) {
        fprintf(stdout, "BAD SERVO VALUE ATTEMPTED!\n");
        return;
    }
    // Write latest servo values
    for(int i = 0; i < NUM_SERVOS; i++){
        servos->ledOnTime(i, 0); // May not need this line?
        servos->ledOffTime(i, servo_values[i]);
    }
}

//
// An extra thread is created and continuosly updates the servo positions
//
void* updateArm(void* data){
    // Variables
    int* future_values = ((servo_thread_struct*)data)->servo_values;
    upm::PCA9685* servos = ((servo_thread_struct*)data)->servos;
    static int mod_values[NUM_SERVOS] = {1,1,1};
    int count = 1;
    bool changed = false;
    bool servosEnabled = true;
    bool okToStart = false;
    int currentSpeed = 7500;

    // Save current servo position goal
    pthread_mutex_lock(&my_lock);
    static int current_values[NUM_SERVOS] = {future_values[0], future_values[1], future_values[2]};
    static int goal_values[NUM_SERVOS] = {future_values[0], future_values[1], future_values[2]};
    pthread_mutex_unlock(&my_lock);

    // Main arm control loop
    while(!gameover) {

        // Determine if arm is enabled
        if(!running && servosEnabled) {
            servosEnabled = false;
            servoEnablePin->write(1);
            okToStart = false;
        } else if (running && !servosEnabled) {
            servosEnabled = true;
            servoEnablePin->write(0);
            usleep(100000);
            okToStart = true;
        }

        // If arm is enabled and turned on
        while(running && okToStart) {
            // Compare and set the current goal position
            for(int i = 0; i < NUM_SERVOS; i++){
                pthread_mutex_lock(&my_lock);
                if(goal_values[i] != future_values[i]){
                    goal_values[0] = future_values[0];
                    goal_values[1] = future_values[1];
                    goal_values[2] = future_values[2];
                    currentSpeed = 10000 - 100 * armSpeed;
                    count = 1;
                    int x_diff = abs(goal_values[0]-current_values[0]);
                    int y_diff = abs(goal_values[1]-current_values[1]);
                    int z_diff = abs(goal_values[2]-current_values[2]);
                    int biggest_diff = max(max(x_diff, y_diff), z_diff);
                    if(x_diff > 0) mod_values[0] = biggest_diff / x_diff;
                    if(y_diff > 0) mod_values[1] = biggest_diff / y_diff;
                    if(z_diff > 0) mod_values[2] = biggest_diff / z_diff;
                    pthread_mutex_unlock(&my_lock);
                    break;
                }
                pthread_mutex_unlock(&my_lock);
            }

            // If we are not currently at the goal then move.
            if(current_values[0] != goal_values[0] || current_values[1] != goal_values[1] || current_values[2] != goal_values[2]) {
                // Check to see which values changed, and adjust accordingly
                changed = false;
                for(int i = 0; i < NUM_SERVOS; i++){
                    // check if it is allowed to move yet
                    if(count % mod_values[i] == 0){
                        if(goal_values[i] > current_values[i]) {
                            if(currentSpeed == 0) {
                                current_values[i] = goal_values[i];
                            }
                            else {
                                current_values[i]++;
                            }
                            changed = true;
                        } else if (goal_values[i] < current_values[i]){
                            if(currentSpeed == 0) {
                                current_values[i] = goal_values[i];
                            }
                            else {
                                current_values[i]--;
                            }
                            changed = true;
                        }
                    }
                }
                count++;
                // Update servos if any values changed
                if(changed) {
                    updateServos(servos, current_values);
                }
                // I think this value will control speed
                usleep(currentSpeed);
            }
        }

    }
    pthread_exit(NULL);
}

//
// Responsible for handling CTRL-C interrupt.
//
void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("[SHUTTING DOWN]\n");
        running = false;
        if(edisonCom != NULL) 
            edisonCom->gameover = true;
        gameover = true;
    }
}

//
// Responsible for determining if the IR Range finder detects a fish or not
//
bool isFishHooked() {
    uint16_t adc_value;
    for(int i = 0; i < 5; i++) {
        adc_value = a0->read();
        fprintf(stdout, "fish test #%d: %d\n", i, adc_value);
        if(adc_value < 288) {
            return false;
        }
        usleep(10000);
    }
    return true;
}

void usage(void) {
    printf("calibrate_arm\n");
    printf("calibrate_cam\n");
    printf("idle\n");
    printf("fish_random\n");
    printf("fish_smart\n");
    printf("custom\n");
    printf("kp:[1-7]\n");
    printf("toss\n");
    printf("load_calibration\n");
    printf("save_calibration\n");
}

//
// Responsible for receiving and parsing input from remote system
// and then saving servo_values in memory.
//
void getCommand(EdisonComm* edisonCom, int servo_values[], upm::PCA9685* servos){
    // Blocking until message is received
    edisonCom->readLine();

    // Analyze serial message
    string commandStr = string(edisonCom->recvBuffer);
    vector<double> pwms;
    stringstream ss(commandStr);
    double i;
    int j;
    string cmd;

    // Get Command type
    if(getline(ss, cmd, ':')){
        if(cmd == "xyz"){
            // Parse command string into pwms array
            while(ss >> i){
                pwms.push_back(i);
                if(ss.peek() == ',')
                    ss.ignore();
            }

            if(pwms.size() == 3){
                double x = pwms.at(0);
                double y = pwms.at(1);
                double z = pwms.at(2);
                int retval = XYZ_to_PWM(x,y,z,servo_values);
            }
        } else if(cmd == "calibrate_arm") {
            // Calibrate arm
            last_state = curr_state;
            curr_state = calibrate_arm;
        } else if(cmd == "calibrate_cam") {
            // Calibrate camera
            last_state = curr_state;
            curr_state = calibrate_cam;
        } else if (cmd == "idle"){
            curr_state = idle;
        } else if(cmd == "fishCheck") {
            fprintf(stdout, "Fish is %shooked\n", isFishHooked() ? "" : "not ");
        } else if(cmd == "custom") {
            last_state = curr_state;
            curr_state = custom;
        } else if(cmd == "kp") {
            ss >> j;
            if(j < 8 && keypointsReceived){
                XYZ_to_PWM(keypoints[j][0],keypoints[j][1],-19,servo_values);
            } else {
                printf("No valid keypoints!\n");
            }
        } else if(cmd == "set_speed") {
            ss >> j;
            if(j <= 100 && j >= 0){
                armSpeed = j;
            } else {
                printf("Invalid Speed!\n");
            }
        } else if(cmd == "toss") {
            tossFish(servos, servo_values);
        } else if(cmd == "grab") {
            ss >> j;
            if(j < 8 && keypointsReceived){
                grabFish(servos, servo_values, j);            
            } else {
                printf("Not a valid keypoint!\n");
            }
            
        } else if(cmd == "fish_random") {
            last_state = curr_state;
            curr_state = fish_random;
        } else if(cmd == "fish_smart") {
            last_state = curr_state;
            curr_state = fish_smart;
        } else if(cmd == "load_calibration") {
            loadCalibration(keypoints);
        } else if(cmd == "save_calibration") {
            saveCalibration(keypoints);
        } else if(cmd == "help") {
            usage();
        } 
    } else {
        fprintf(stderr, "Error: No command found in message\n");
        return;
    }
    return;
}

//---------------------------------- SETUP ----------------------------------//

//
// Responsible for setting up the multiple threads active in program
//
void setupMultithreading(pthread_t* servo_control_thread, pthread_t* state_control_thread, servo_thread_struct* data) {
    int servo_thread_retval = pthread_create(servo_control_thread, NULL, updateArm, (void *)data);
    int state_thread_retval = pthread_create(state_control_thread, NULL, updateState, (void *)data);
    if(state_thread_retval || servo_thread_retval){
        fprintf(stderr, "Error: Unable to create thread\n");
        exit(-1);
    }
}

//
// Responsible for parsing command line arguements and determining
// which mode to run in.
//
int setupEnvironment(int argc, char* argv[], int* mode){
    // Example for reference: http://www.gnu.org/software/libc/manual/html_node/Example-of-Getopt.html#Example-of-Getopt
    int index;
    int c;

    opterr = 0;
    while ((c = getopt (argc, argv, "wt")) != -1){
        switch (c){
          case 'w':
            *mode = WIRELESS_MODE;
            break;
          case 't':
            *mode = TEST_MODE;
            break;
          case '?':
            if (isprint (optopt))
              fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
              fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
            return 1;
          default:
            abort ();
        }
    }

    // Initialize ADC
    a0 = new mraa::Aio(0);
    if(a0 == NULL){
        fprintf(stderr, "Could not Initialize ADC\n");
        exit(-1);
    }

    servoEnablePin = new mraa::Gpio(6);
    if(servoEnablePin == NULL){
        fprintf(stderr, "cant init servo pin\n");
        exit(-1);
    }
    servoEnablePin->dir(mraa::DIR_OUT);
    servoEnablePin->write(1);

    srand(time(NULL));

    return 0;
}

//
// Responsible for initializing which ever COM mode user decides to use.
//
void setupComms(int mode){
  edisonCom = EdisonComm::initComm(mode);
  cognexCom = new CognexSerial;
  return;
}

//
// Responsible for setting up all system interrupts.
//
void setupInterrupts(){
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);

    // Setup ISR trigger on both edges
    //mraa::Gpio* x = new mraa::Gpio(7);
    //x->dir(mraa::DIR_IN);
    //x->isr(mraa::EDGE_BOTH, &toggleState, NULL);

    fprintf(stdout, "INTERRUPTS: RUNNING\n");
    fprintf(stdout, "GAMESTATE:  %s\n", running ? "RUNNING" : "NOT RUNNING");
}

//
// Responsible for settign up PWM channels.
//
int setupPwm(upm::PCA9685 **servos){
    servoEnablePin->write(1);
    (*servos) = new upm::PCA9685(PCA9685_I2C_BUS,PCA9685_DEFAULT_I2C_ADDR);
    // put device to sleep
    (*servos)->setModeSleep(true);
    // setup a period of 50Hz
    (*servos)->setPrescaleFromHz(190);
    // wake device up
    (*servos)->setModeSleep(false);

    (*servos)->ledOnTime(PCA9685_ALL_LED, 0);
    (*servos)->ledOffTime(PCA9685_ALL_LED, DEFAULT_SCALED_PWM);

    sleep(1);

    for(int i = 0; i < NUM_SERVOS; i++){
        (*servos)->ledOnTime(i, 0);
        (*servos)->ledOffTime(i, DEFAULT_SCALED_PWM);
    }
    return 0;
}

//
// Main 
//
int main(int argc, char* argv[]) { 
    // Variables
    int servo_values[NUM_SERVOS] = {DEFAULT_SCALED_PWM,DEFAULT_SCALED_PWM,DEFAULT_SCALED_PWM};
    int mode = CONSOLE_MODE; //DEFAULT
    upm::PCA9685 *servos;
    int retval;
    int rPtr;
    bool notRunYet = true;
    pthread_t servo_control_thread;
    pthread_t state_control_thread;

    // Setup
    setupInterrupts();
    setupEnvironment(argc, argv, &mode);
    setupComms(mode);
    setupPwm(&servos);
    servo_thread_struct data = {servos, servo_values};
    setupMultithreading(&servo_control_thread, &state_control_thread, &data);

    // Main Loop
    while(!gameover){
        getCommand(edisonCom, servo_values, servos);
    }

    // Wait until update thread exits successfully
    pthread_join(servo_control_thread, (void**)&rPtr);
    pthread_join(state_control_thread, (void**)&rPtr);
    fprintf(stdout, "All threads closed successfully!\n");
    
    // Clean up memory
    delete servos;
    if(mode == SERIAL_MODE)
        delete edisonCom; 
    if(cognexCom != NULL){
        delete cognexCom;
    }
    if(a0 != NULL){
        delete a0;
    }

    return MRAA_SUCCESS;
}
