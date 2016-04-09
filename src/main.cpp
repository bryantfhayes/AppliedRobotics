/* File: main.cpp
 * Author: Bryant Hayes
 * Description: Low level code for controlling robotic arm based on receieved serial commands
 */

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include "mraa.hpp"
#include <string.h>
#include "EdisonComm.h"
#include <time.h>

//
// DEFINES
//
#define TRUE 1
#define FALSE 0
#define SERIAL_PORT "/dev/ttyMFD1"
#define NUM_SERVOS 4
#define SERVO_PERIOD 20000
#define DEFAULT_PWM 1426
#define MSG_SIZE 32
#define SERIAL_MODE 0
#define WIRELESS_MODE 1

using namespace std;

//
// GLOBAL VARIABLES
//
const int servo_pins[NUM_SERVOS] = {3,5,6,9};
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
}

//
// Responsible for settign up PWM channels.
//
int setupPwm(mraa::Pwm* servos[]){
    for(int i = 0; i < NUM_SERVOS; i++){
        servos[i] = new mraa::Pwm(servo_pins[i]);
        servos[i]->period_us(SERVO_PERIOD);
        // Check that instantiation was successful
        if(servos[i] == NULL)
            return MRAA_ERROR_UNSPECIFIED;
        servos[i]->enable(true);
        servos[i]->pulsewidth_us(DEFAULT_PWM);
    }

    fprintf(stdout, "PWM:        RUNNING\n");
    return MRAA_SUCCESS;
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

    fprintf(stdout, "Got a message: %s\n", com->recvBuffer);

    // Analyze serial message
    string commandStr = string(com->recvBuffer);
    vector<int> pwms;
    stringstream ss(commandStr);
    int i;

    // Parse command string into pwms array
    while(ss >> i){
        pwms.push_back(i);
        if(ss.peek() == ',')
            ss.ignore();
    }

    for(int i = 0; i < pwms.size(); i++){
        servo_values[i] = pwms.at(i);
    	fprintf(stdout, "Servo #%d = %d\n", i, servo_values[i]);
    }
}

//
// Responsible for setting PWM duty cycle to control servo position.
//
void updateServos(mraa::Pwm* servos[], int servo_values[]){
    // Write latest servo values
    for(int i = 0; i < NUM_SERVOS; i++){
        servos[i]->pulsewidth_us(servo_values[i]);
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
    while ((c = getopt (argc, argv, "w")) != -1){
        switch (c){
          case 'w':
            *mode = WIRELESS_MODE;
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
}

//
// Main 
//
int main(int argc, char* argv[]) {
    // Variables
    int servo_values[NUM_SERVOS];
    int mode = SERIAL_MODE; //DEFAULT
    mraa::Pwm* servos[NUM_SERVOS];
    
    // Setup
    setupInterrupts();
    setupEnvironment(argc, argv, &mode);
    setupComms(mode);
    setupPwm(servos);

    // Main Loop
    while(!gameover){
        // Game control loop
        while(running){
            getCommand(com, servo_values);
            updateServos(servos, servo_values);
        }
    }

    // Clean up memory
    if(mode == SERIAL_MODE)
        delete com;
    for(int i = 0; i < NUM_SERVOS; i++){
        delete servos[i];
    }

    return MRAA_SUCCESS;
}
