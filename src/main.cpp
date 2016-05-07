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

//
// GLOBAL VARIABLES
//
static bool running = false;
static bool gameover = false;
static time_t last_interrupt_time = 0;
static EdisonComm* com;
pthread_mutex_t my_lock;
mraa::Aio* a0;


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
// Responsible for handling toggle of INTERRUPT_PIN
// which then starts/stops the game.
//
void toggleState(void* args){
    time_t interrupt_time = time(0);
    if (difftime(interrupt_time, last_interrupt_time) >= 1) {
        running = !running;
        //com->gameover = gameover;
        fprintf(stdout, "GAMESTATE:  %s\n", running ? "RUNNING" : "NOT RUNNING");
    }
    last_interrupt_time = interrupt_time;
}

//
// Responsible for setting up all system interrupts.
//
void setupInterrupts(){
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);

    // Setup ISR trigger on both edges
    mraa::Gpio* x = new mraa::Gpio(7);
    x->dir(mraa::DIR_IN);
    x->isr(mraa::EDGE_BOTH, &toggleState, NULL);

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

void calibrate(void) {
    fprintf(stdout, "calibrating...\n");
    fprintf(stdout, "calibration complete!\n");
}

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

#ifdef DEBUG
    fprintf(stdout, "Got a message: %s\n", com->recvBuffer);
#endif

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
                if(retval == -1){
#ifdef DEBUG
                    fprintf(stdout, "\tReceived unreachable coordinates\n");
#endif
                } else {
                    for(int i = 0; i < NUM_SERVOS; i++){
#ifdef DEBUG
                    fprintf(stdout, "\tServo #%d = %d\n", i+1, servo_values[i]);
#endif
                    }
                }
            }
        } else if(cmd == "calibrate") {
            // Calibrate arm and get board orientation
            calibrate();
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
    while(!gameover) {
        if(!running && servosEnabled) {
            servosEnabled = false;
            servos->setModeSleep(true);
        } else if (running && !servosEnabled) {
            servosEnabled = true;
            servos->setModeSleep(false);
            usleep(100000);
        }
        while(running) {
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
                //fprintf(stdout,"UPDATE!\n");
            }
            // I think this value will control speed
            usleep(7500);
        }

    }
    pthread_exit(NULL);
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

void testFunction(){
    uint16_t adc_value;
    float adc_value_float;
/*
    while (!gameover) {
        adc_value = a0->read();
        adc_value_float = a0->readFloat();
        fprintf(stdout, "ADC A0 Read %X - %d\n", adc_value, adc_value);
        fprintf(stdout, "ADC A0 Float Read - %.5f\n", adc_value_float);
        usleep(50000);
    }
*/
    
    printf("Waiting for serial message\n");
    com = EdisonComm::initComm(SERIAL_MODE);
    while(!gameover){  
        com->readLine();
        printf("Received Message: %s\n", com->recvBuffer);
        printf("Sending: HELLO WORLD\n");
        com->writeLine("HELLO WORLD", 11);
    }
    delete com;
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
    pthread_t thread;

    // Setup
    setupInterrupts();
    setupEnvironment(argc, argv, &mode);

    testFunction();
    exit(1);

    setupComms(mode);
    //setupPwm(&servos);

    // Main Loop
    while(!gameover){
        getCommand(com, servo_values);
        if(running && notRunYet){
            notRunYet = false;
            setupPwm(&servos);
            // Create a new thread to handle servo updates
            servo_thread_struct data = {servos, servo_values};
            retval = pthread_create(&thread, NULL, updateThread, (void *)&data);
            if(retval) {
                fprintf(stderr, "Error: Unable to create thread\n");
                exit(-1);
            }
        }
    }

    // Join thread if it was created
    if(!notRunYet){
        // Wait until update thread exits successfully
        pthread_join(thread, (void**)&rPtr);
        fprintf(stdout, "\n return value from update thread is [%d]\n", rPtr);
    }

    // Clean up memory
    delete servos;
    if(mode == SERIAL_MODE)
        delete com; 
    if(a0 != NULL){
        delete a0;
    }

    return MRAA_SUCCESS;
}
