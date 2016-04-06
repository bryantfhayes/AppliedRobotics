/*
 * Author: Thomas Ingleby <thomas.c.ingleby@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "mraa.hpp"

int running = 0;

using namespace std;

void
sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = -1;
    }
}

int
main()
{
//! [Interesting]
    // If you have a valid platform configuration use numbers to represent uart
    // device. If not use raw mode where std::string is taken as a constructor
    // parameter
    mraa::Uart* dev;
    try {
        dev = new mraa::Uart(0);
    } catch (std::exception& e) {
        std::cout << e.what() << ", likely invalid platform config" << std::endl;
    }

    try {
        dev = new mraa::Uart("/dev/ttyMFD1");
    } catch (std::exception& e) {
        std::cout << "Error while setting up raw UART, do you have a uart?" << std::endl;
        std::terminate();
    }

    if (dev->setBaudRate(115200) != 0) {
        std::cout << "Error setting parity on UART" << std::endl;
    }

    if (dev->setMode(8, MRAA_UART_PARITY_NONE, 1) != 0) {
        std::cout << "Error setting parity on UART" << std::endl;
    }

    if (dev->setFlowcontrol(false, false) != 0) {
        std::cout << "Error setting flow control UART" << std::endl;
    }

    dev->writeStr("Hello monkeys\n");
    dev->writeStr("SECOND LINE\n");
    //! [Interesting]
while(1){
    char command[16];
    while(1){
	char buffer[16];
	char *buffer_ptr = buffer;
	if(dev->dataAvailable()){
		dev->read(command, 16); 
		command[strlen(command)-1] = '\0';
		break;
	}
    }
	string str = string(command);
	vector<int> vect;

	stringstream ss(str);
	int i;

	while(ss >> i){
		vect.push_back(i);
		if(ss.peek() == ',')
			ss.ignore();
	}
	for(int i = 0; i < vect.size(); i++){
		cout << vect.at(i) << endl;
	}

}
    delete dev;





    signal(SIGINT, sig_handler);
    //! [Interesting]
    mraa::Pwm* pwm1;
    mraa::Pwm* pwm2;
    mraa::Pwm* pwm3;
    mraa::Pwm* pwm4;

    pwm1 = new mraa::Pwm(3);
    pwm2 = new mraa::Pwm(5);
    pwm3 = new mraa::Pwm(6);
    pwm4 = new mraa::Pwm(9);
 
    pwm1->period_us(20000);
    pwm2->period_us(20000);
    pwm3->period_us(20000);
    pwm4->period_us(20000);
 
    if (pwm1 == NULL || pwm2 == NULL || pwm3 == NULL || pwm4 == NULL) {
        return MRAA_ERROR_UNSPECIFIED;
    }
    fprintf(stdout, "Cycling PWM on IO3 (pwm3) \n");
    pwm1->enable(true);
    pwm2->enable(true);
    pwm3->enable(true);
    pwm4->enable(true);
    
    pwm1->pulsewidth_us(1426);
    pwm2->pulsewidth_us(1426);
    pwm3->pulsewidth_us(1426);
    pwm4->pulsewidth_us(1426);
    float value = 0.0f;
    int angle = 0;
    while (running == 0) {
	cout << "Enter angle:\n";
	cin >> angle;
	cout << "You entered: \n" << angle;
        pwm3->pulsewidth_us(angle);
	//value = value + 0.01f;
        //pwm->write(value);
        //usleep(50000);
        //if (value >= 1.0f) {
        //    value = 0.0f;
        //}
    }
    delete pwm1;
    delete pwm2;
    delete pwm3;
    delete pwm4;
    //! [Interesting]

    return MRAA_SUCCESS;
}
