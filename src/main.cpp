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
#define DEFAULT_PWM 1245
#define MSG_SIZE 32
#define SERIAL_MODE 0
#define WIRELESS_MODE 1
#define CONSOLE_MODE 2
#define TEST_MODE 3

// Servo safety bounds
#define SERVO_X_MIN 700
#define SERVO_X_MAX 1750
#define SERVO_Y_MIN 750
#define SERVO_Y_MAX 1375
#define SERVO_Z_MIN 750
#define SERVO_Z_MAX 1600

#define DEBUG 1

using namespace std;

//
// GLOBAL VARIABLES
//
static bool running = true;
static bool gameover = false;
static time_t last_interrupt_time = 0;
static EdisonComm* com;

//
// Responsible for handling CTRL-C interrupt.
//
void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("[SHUTTING DOWN]\n");
        running = false;
        if(com != NULL) 
            com->running = false;
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
        com->running = running;
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
    mraa::Gpio* x = new mraa::Gpio(4);
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
    (*servos)->ledOffTime(PCA9685_ALL_LED, DEFAULT_PWM);


    sleep(1);

    for(int i = 0; i < NUM_SERVOS; i++){
        (*servos)->ledOnTime(i, 0);
        (*servos)->ledOffTime(i, DEFAULT_PWM);
    }
    return 0;
}

//
// Responsible for receiving and parsing input from remote system
// and then saving servo_values in memory.
//
void getCommand(EdisonComm* com, int servo_values[]){

    // Blocking until message is received
    com->readLine();

    // Return if running is FALSE
    if(!running) return;
#ifdef DEBUG
    fprintf(stdout, "Got a message: %s\n", com->recvBuffer);
#endif

    // Analyze serial message
    string commandStr = string(com->recvBuffer);
    vector<double> pwms;
    stringstream ss(commandStr);
    double i;

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
// Main 
//
int main(int argc, char* argv[]) { 
    // Variables
    int servo_values[NUM_SERVOS];
    int mode = CONSOLE_MODE; //DEFAULT
    upm::PCA9685 *servos;
    
    // Setup
    setupInterrupts();
    setupEnvironment(argc, argv, &mode);
    setupComms(mode);
    setupPwm(&servos);

    // TEST CODE HERE
    if(mode == TEST_MODE){
        printf("TEST CODE ACTIVE\n");
        int test_values[NUM_SERVOS] = {1244, 1244, 1244};
        updateServos(servos, test_values);       

        sleep(1);

        char input[MSG_SIZE];
        while(!gameover){
            printf("Enter PWM values [a,b,c]:");
            scanf("%s", input);

            // Analyze serial message
            string commandStr = string(input);
            vector<double> pwms;
            stringstream ss(commandStr);
            double i;

            // Parse command string into pwms array
            while(ss >> i){
                pwms.push_back(i);
                if(ss.peek() == ',')
                    ss.ignore();
            }

            if(pwms.size() == 3){
                /*
                double g, a, b;
                g = (double)pwms.at(0);
                a = (double)pwms.at(1);
                b = (double)pwms.at(2);
                // Determine PWM values
                angle_to_pwm(g, a, b, test_values);
                // map pwm values to range required for 1/4096 resolution
                scale_pwm(test_values);
                */
                
                test_values[0] = (double)pwms.at(0);
                test_values[1] = (double)pwms.at(1);
                test_values[2] = (double)pwms.at(2);
                
                updateServos(servos, test_values);
            }
            
        }

        // Clean up memory
        delete servos;
        if(mode == SERIAL_MODE)
            delete com; 
        return 0;
    }

    // Main Loop
    while(!gameover){
        // Game control loop
        while(running){
            getCommand(com, servo_values);
            updateServos(servos, servo_values);
        }
    }

    // Clean up memory
    delete servos;
    if(mode == SERIAL_MODE)
        delete com; 

    return MRAA_SUCCESS;
}
