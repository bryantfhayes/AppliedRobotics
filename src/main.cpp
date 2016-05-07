//
// File: main.cpp
// Author: Bryant Hayes
// Description: Low level code for controlling robotic arm based on receieved serial commands
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
#define SERVO_X_MIN 850
#define SERVO_X_MAX 1750
#define SERVO_Y_MIN 750
#define SERVO_Y_MAX 1375
#define SERVO_Z_MIN 750
#define SERVO_Z_MAX 1600

//#define DEBUG 1

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
    idle, calibrate
};

//
// GLOBAL VARIABLES
//
static bool running = false;
static bool gameover = false;
static time_t last_interrupt_time = 0;
static EdisonComm* com;
pthread_mutex_t my_lock;
mraa::Aio* a0;
state_t curr_state = idle;


void idle_state_func(void* data) {
    printf("idle state!\n");
    running = false;
    while(curr_state == idle && !gameover){}
    return;
}

void calibrate_state_func(void* data) {
    int* servo_values = ((servo_thread_struct*)data)->servo_values;
    running = true;
    XYZ_to_PWM(0,39,-23,servo_values);
    while(curr_state == calibrate && !gameover){
        if(curr_state != calibrate) break;
        usleep(5000000);
        XYZ_to_PWM(-10,39,-23,servo_values);
        if(curr_state != calibrate) break;
        usleep(7500000);
        XYZ_to_PWM(10,39,-23,servo_values);
        if(curr_state != calibrate) break;
        usleep(5000000);
        XYZ_to_PWM(0,39,-23,servo_values);
        
    }
    return;
}

//
// Monitor state and update internal state machine to drive gameplay
//
void* updateState(void* data){
    void (* state[])(void*) = {idle_state_func, calibrate_state_func};
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
void* updateThread(void* data){
    int* future_values = ((servo_thread_struct*)data)->servo_values;
    upm::PCA9685* servos = ((servo_thread_struct*)data)->servos;
    pthread_mutex_lock(&my_lock);
    static int current_values[NUM_SERVOS] = {future_values[0], future_values[1], future_values[2]};
    static int goal_values[NUM_SERVOS] = {future_values[0], future_values[1], future_values[2]};
    pthread_mutex_unlock(&my_lock);
    static int mod_values[NUM_SERVOS] = {1,1,1};
    int count = 1;
    bool changed = false;
    bool servosEnabled = true;
    bool okToStart = false;
    while(!gameover) {
        if(!running && servosEnabled) {
            servosEnabled = false;
            servos->setModeSleep(true);
            okToStart = false;
        } else if (running && !servosEnabled) {
            servosEnabled = true;
            servos->setModeSleep(false);
            usleep(100000);
            okToStart = true;
        }
        while(running && okToStart) {
            // Compare and set the current goal position
            for(int i = 0; i < NUM_SERVOS; i++){
                pthread_mutex_lock(&my_lock);
                if(goal_values[i] != future_values[i]){
                    goal_values[0] = future_values[0];
                    goal_values[1] = future_values[1];
                    goal_values[2] = future_values[2];
                    count = 1;
                    int x_diff = abs(goal_values[0]-current_values[0]);
                    //fprintf(stdout, "goal: %d");
                    int y_diff = abs(goal_values[1]-current_values[1]);
                    int z_diff = abs(goal_values[2]-current_values[2]);
                    int biggest_diff = max(max(x_diff, y_diff), z_diff);
                    if(x_diff > 0) mod_values[0] = biggest_diff / x_diff;
                    if(y_diff > 0) mod_values[1] = biggest_diff / y_diff;
                    if(z_diff > 0) mod_values[2] = biggest_diff / z_diff;
                    //fprintf(stdout, "diffs- x:%d, y:%d, z:%d\n", x_diff, y_diff, z_diff);
                    //fprintf(stdout, "mods- x:%d, y:%d, z:%d\n", mod_values[0], mod_values[1], mod_values[2]);
                    pthread_mutex_unlock(&my_lock);
                    break;
                }
                pthread_mutex_unlock(&my_lock);
            }

            // Check to see if any values changed, and adjust
            changed = false;
            for(int i = 0; i < NUM_SERVOS; i++){
                // check if it is allowed to move yet
                if(count % mod_values[i] == 0){
                    if(goal_values[i] > current_values[i]) {
                        current_values[i]++;
                        changed = true;
                    } else if (goal_values[i] < current_values[i]){
                        current_values[i]--;
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
            usleep(7500);
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
        if(com != NULL) 
            com->gameover = true;
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

//
// Responsible for receiving and parsing input from remote system
// and then saving servo_values in memory.
//
void getCommand(EdisonComm* com, int servo_values[]){

    // Blocking until message is received
    com->readLine();

    // Analyze serial message
    string commandStr = string(com->recvBuffer);
    vector<double> pwms;
    stringstream ss(commandStr);
    double i;
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
        } else if(cmd == "calibrate") {
            // Calibrate arm and get board orientation
            curr_state = calibrate;
        } else if (cmd == "idle"){
            curr_state = idle;
        } else if(cmd == "off") {
            running = false;
            fprintf(stdout, "Turning off servos...\n");
        } else if(cmd == "on") {
            running = true;
            fprintf(stdout, "Turning on servos...\n");
        } else if(cmd == "fishCheck") {
            fprintf(stdout, "Fish is %shooked\n", isFishHooked() ? "" : "not ");
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
    int servo_thread_retval = pthread_create(servo_control_thread, NULL, updateThread, (void *)data);
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

    return 0;
}

//
// Responsible for initializing which ever COM mode user decides to use.
//
void setupComms(int mode){
  com = EdisonComm::initComm(mode);
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
    (*servos) = new upm::PCA9685(PCA9685_I2C_BUS,PCA9685_DEFAULT_I2C_ADDR);
    // put device to sleep
    (*servos)->setModeSleep(true);
    // setup a period of 50Hz
    (*servos)->setPrescaleFromHz(190);
    // wake device up
    // (*servos)->setModeSleep(false);

    // (*servos)->ledOnTime(PCA9685_ALL_LED, 0);
    // (*servos)->ledOffTime(PCA9685_ALL_LED, DEFAULT_SCALED_PWM);

    // sleep(1);

    // for(int i = 0; i < NUM_SERVOS; i++){
    //     (*servos)->ledOnTime(i, 0);
    //     (*servos)->ledOffTime(i, DEFAULT_SCALED_PWM);
    // }
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
        getCommand(com, servo_values);
    }

    // Wait until update thread exits successfully
    pthread_join(servo_control_thread, (void**)&rPtr);
    pthread_join(state_control_thread, (void**)&rPtr);
    fprintf(stdout, "All threads closed successfully!\n");
    
    // Clean up memory
    delete servos;
    if(mode == SERIAL_MODE)
        delete com; 
    if(a0 != NULL){
        delete a0;
    }

    return MRAA_SUCCESS;
}
