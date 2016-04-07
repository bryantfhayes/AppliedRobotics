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

//
// DEFINES
//
#define TRUE 1
#define FALSE 0
#define SERIAL_PORT "/dev/ttyMFD2"
#define NUM_SERVOS 4
#define SERVO_PERIOD 20000
#define DEFAULT_PWM 1426
#define MSG_SIZE 32


//
// GLOBAL VARIABLES
//
const int servo_pins[NUM_SERVOS] = {3,5,6,9};
int running = TRUE;

using namespace std;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("Shutting down cleanly...\n");
        running = FALSE;
    }
}

void setupInterrupts(){
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);
    fprintf(stdout, "INTERRUPTS: RUNNING\n");
}

int setupUart(mraa::Uart** com){
    try {
        *com = new mraa::Uart(0);
    } catch (std::exception& e) {
        std::cout << e.what() << ", likely invalid platform config" << std::endl;
    }

    try {
        *com = new mraa::Uart(SERIAL_PORT);
    } catch (std::exception& e) {
        std::cout << "Error while setting up raw UART, do you have a uart?" << std::endl;
        std::terminate();
    }

    if ((*com)->setBaudRate(115200) != 0) {
        std::cout << "Error setting parity on UART" << std::endl;
    }

    if ((*com)->setMode(8, MRAA_UART_PARITY_NONE, 1) != 0) {
        std::cout << "Error setting parity on UART" << std::endl;
    }

    if ((*com)->setFlowcontrol(false, false) != 0) {
        std::cout << "Error setting flow control UART" << std::endl;
    }

    fprintf(stdout, "UART:       RUNNING\n");
    return MRAA_SUCCESS;

    // EXAMPLE USAGE:
    // com->writeStr("Hello World\n");
    // com->readStr(16);
}

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

void getCommand(mraa::Uart* com, int servo_values[]){
    char command[MSG_SIZE];

    // Wait for message
    while(running == TRUE){
        if(com->dataAvailable()){
            com->read(command, MSG_SIZE);
            break;
        }
    }

    // Return if running is FALSE
    if(!running) return;

    fprintf(stdout, "Got a message: %s\n", command);

    // Analyze serial message
    string commandStr = string(command);
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

void updateServos(mraa::Pwm* servos[], int servo_values[]){
    // Write latest servo values
    for(int i = 0; i < NUM_SERVOS; i++){
        servos[i]->pulsewidth_us(servo_values[i]);
    }
}

int main(int argc, char* argv[]) {
    // Variables
    mraa::Uart* com;
    mraa::Pwm* servos[NUM_SERVOS];
    int servo_values[NUM_SERVOS];

    // Setup
    setupInterrupts();
    setupUart(&com);
    setupPwm(servos);

    // Main Loop
    while(running == TRUE){
        getCommand(com, servo_values);
        updateServos(servos, servo_values);
    }

    // Clean Up
    delete com;
    for(int i = 0; i < NUM_SERVOS; i++){
        delete servos[i];
    }

    return MRAA_SUCCESS;
}
