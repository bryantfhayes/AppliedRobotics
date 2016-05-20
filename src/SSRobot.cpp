/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 13:21:56
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-19 20:28:08
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <pthread.h>

#include "SSRobot.h"
#include "BHUtils.h"

using namespace std;

// --- PUBLIC METHODS --- //

//
// Constructor
//
SSRobot::SSRobot(){
	_alive = true;
	_armSpeed = 50;
	_currentPWM = {DEFAULT_PWM, DEFAULT_PWM, DEFAULT_PWM};
	_targetPWM = {DEFAULT_PWM, DEFAULT_PWM, DEFAULT_PWM};
	if (initServos() || initControlThread()) {
		throw 20;
	}


}

//
// Destructor
//
SSRobot::~SSRobot(){
	_alive = false;
	enable(false);
	fprintf(stdout, "Terminating robot control thread...\n");
	pthread_join(_controlThread, (void**)&_rPtr);
	delete _servos.enablePin;
	delete _servos.controller;
}

//
// Enable the pin to active power to servos
//
void SSRobot::enable(bool state) {
	if (!state) {
		_servos.enablePin->write(1);
	} else {
		_servos.enablePin->write(0);
	}
}

//
// Set the movement speed of the arm (1-100), where 100 is 
// max speed (point-to-point).
//
void SSRobot::setSpeed(int speed) {
	if (speed <= 100 && speed > 0) {
		_armSpeed = speed;
	} else {
		fprintf(stderr, "ERROR: Invalid speed\n");
	}
	
}

//
// Get the current arm movement speed
//
int SSRobot::getSpeed() {
	return _armSpeed;
}

//
// Set the XYZ position of the arm
//
int SSRobot::setPosition(double x, double y, double z) {
	// Determine required joint angles
    double g = calculateGamma(x,y,z);
    double t = degs_to_rads(g);
    double a = calculateAlpha(x-(TOOL_LENGTH*sin(t)),y-(TOOL_LENGTH*cos(t)),z+TOOL_HEIGHT);
    double b = calculateBeta(x-(TOOL_LENGTH*sin(t)),y-(TOOL_LENGTH*cos(t)),z+TOOL_HEIGHT);
    
    // Check to see if all values were calculated successfully
    if(std::isnan(a) || std::isnan(b) || std::isnan(g)){
    	fprintf(stderr, "ERROR: Invalid Position\n");
        return 1;
    }

    _lastXYZ = _targetXYZ;
    _targetXYZ = {x,y,z};
    _targetPWM = angleToPwm(g, a, b);

    if (_armSpeed < 100){
        usleep(750000);
    } else {
        usleep(750000 + biggestDifference(_currentPWM, _targetPWM) * (10000 - (100 *_armSpeed)));
    }

    return 0;
}

int SSRobot::setPosition(double x, double y, double z, int delay) {
// Determine required joint angles
    double g = calculateGamma(x,y,z);
    double t = degs_to_rads(g);
    double a = calculateAlpha(x-(TOOL_LENGTH*sin(t)),y-(TOOL_LENGTH*cos(t)),z+TOOL_HEIGHT);
    double b = calculateBeta(x-(TOOL_LENGTH*sin(t)),y-(TOOL_LENGTH*cos(t)),z+TOOL_HEIGHT);
    
    // Check to see if all values were calculated successfully
    if(std::isnan(a) || std::isnan(b) || std::isnan(g)){
        fprintf(stderr, "ERROR: Invalid Position\n");
        return 1;
    }

    _lastXYZ = _targetXYZ;
    _targetXYZ = {x,y,z};
    _targetPWM = angleToPwm(g, a, b);

    usleep(delay);

    return 0;
}

//
// Get the current state of the robot. Used to kill extra threads
// upon termination.
//
bool SSRobot::isAlive() {
	return _alive;
}

// --- PROTECTED METHODS --- //

int SSRobot::biggestDifference(pwms_t a, pwms_t b) {
    int x_diff, y_diff, z_diff, biggest_diff;
    x_diff = abs(a.x - b.x);
    y_diff = abs(a.y - b.y);
    z_diff = abs(a.z - b.z);
    biggest_diff = max(max(x_diff, y_diff), z_diff);
    return biggest_diff;
}

//
// Continuously executes in its own thread to update arm movements.
// Relies on monitoring _targetPWM and it's difference from _currentPWM
//
void* SSRobot::update(void* context) {
	SSRobot* robot = ((SSRobot*)context);
	int currentSpeed, count, x_diff, y_diff, z_diff, biggest_diff;
	pwms_t goalPWM = {robot->_targetPWM.x, robot->_targetPWM.y, robot->_targetPWM.z};
	pwms_t modValues = {1,1,1};
	bool changed = false;

	while (robot->isAlive()) {
		if (!(robot->_targetPWM == goalPWM)) {
			goalPWM.x = robot->_targetPWM.x;
            goalPWM.y = robot->_targetPWM.y;
            goalPWM.z = robot->_targetPWM.z;
			currentSpeed = 10000 - 100 * robot->_armSpeed;
			count = 1;
			x_diff = abs(goalPWM.x - robot->_currentPWM.x);
			y_diff = abs(goalPWM.y - robot->_currentPWM.y);
			z_diff = abs(goalPWM.z - robot->_currentPWM.z);
			biggest_diff = max(max(x_diff, y_diff), z_diff);
			if(x_diff > 0) modValues.x = biggest_diff / x_diff;
            if(y_diff > 0) modValues.y = biggest_diff / y_diff;
            if(z_diff > 0) modValues.z = biggest_diff / z_diff;
		}

		// This doesnt follow DRY principle. Could possibly iterate over struct using 
		// a pointer and increase offset by sizeof(int).
		if (!(goalPWM == robot->_currentPWM)) {
			changed = false;
			// X
			if(count % modValues.x == 0){
                if(goalPWM.x > robot->_currentPWM.x) {
                    if(currentSpeed == 0) {
                        robot->_currentPWM.x = goalPWM.x;
                    }
                    else {
                        robot->_currentPWM.x++;
                    }
                    changed = true;
                } else if (goalPWM.x < robot->_currentPWM.x){
                    if(currentSpeed == 0) {
                        robot->_currentPWM.x = goalPWM.x;
                    }
                    else {
                        robot->_currentPWM.x--;
                    }
                    changed = true;
                }
            }

            // Y
            if(count % modValues.y == 0){
                if(goalPWM.y > robot->_currentPWM.y) {
                    if(currentSpeed == 0) {
                        robot->_currentPWM.y = goalPWM.y;
                    }
                    else {
                        robot->_currentPWM.y++;
                    }
                    changed = true;
                } else if (goalPWM.y < robot->_currentPWM.y){
                    if(currentSpeed == 0) {
                        robot->_currentPWM.y = goalPWM.y;
                    }
                    else {
                        robot->_currentPWM.y--;
                    }
                    changed = true;
                }
            }

            // Z
            if(count % modValues.z == 0){
                if(goalPWM.z > robot->_currentPWM.z) {
                    if(currentSpeed == 0) {
                        robot->_currentPWM.z = goalPWM.z;
                    }
                    else {
                        robot->_currentPWM.z++;
                    }
                    changed = true;
                } else if (goalPWM.z < robot->_currentPWM.z){
                    if(currentSpeed == 0) {
                        robot->_currentPWM.z = goalPWM.z;
                    }
                    else {
                        robot->_currentPWM.z--;
                    }
                    changed = true;
                }
            }

            count++;

            if (changed) {
            	robot->_servos.xValue = robot->_currentPWM.x;
            	robot->_servos.yValue = robot->_currentPWM.y;
            	robot->_servos.zValue = robot->_currentPWM.z;
            	robot->updateServos();
            }
            if(currentSpeed > 0) {
                usleep(currentSpeed);
            }
            

		}
	}
	pthread_exit(NULL);
}

//
// Forks a new thread to control movement of the arm
//
int SSRobot::initControlThread() {
	_rPtr = pthread_create(&_controlThread, NULL, SSRobot::update, this);
	if(_rPtr){
        fprintf(stderr, "ERROR: Unable to create control thread\n");
        return 1;
    } else {
    	return 0;
    }
}

//
// Initialize and write default values to the arm
//
int SSRobot::initServos() {
	// Setup servo enable pin on GPIO Pin
	_servos.enablePin = new mraa::Gpio(SERVO_ENABLE_PIN);
    if(_servos.enablePin == NULL){
        fprintf(stderr, "ERROR: Setting up servo enable pin.\n");
        return 1;
    }
    _servos.enablePin->dir(mraa::DIR_OUT);
    _servos.enablePin->write(1);

    usleep(1000000);

    // Setup pwm controller
    _servos.controller = new upm::PCA9685(PCA9685_I2C_BUS,PCA9685_DEFAULT_I2C_ADDR);
    if(_servos.controller == NULL){
        fprintf(stderr, "ERROR: Setting up PCA9685 device.\n");
        return 1;
    }
    _servos.controller->setModeSleep(true);
    _servos.controller->setPrescaleFromHz(PWM_HZ - 10);
    _servos.controller->setModeSleep(false);
    _servos.controller->ledOnTime(PCA9685_ALL_LED, 0);
    _servos.controller->ledOffTime(PCA9685_ALL_LED, DEFAULT_PWM);
    //sleep(1);

    for(int i = 0; i < NUM_SERVOS; i++){
        _servos.controller->ledOnTime(i, 0);
        _servos.controller->ledOffTime(i, DEFAULT_PWM);
    }
    return 0;
}

//
// Responsible for setting PWM duty cycle to control servo position.
//
int SSRobot::updateServos() {
    // Safety range check
    if (((_servos.xValue > SERVO_X_MAX) || (_servos.xValue < SERVO_X_MIN)) || ((_servos.yValue > SERVO_Y_MAX) || (_servos.yValue < SERVO_Y_MIN)) || ((_servos.zValue > SERVO_Z_MAX) || (_servos.zValue < SERVO_Z_MIN))) {
        fprintf(stdout, "BAD SERVO VALUE ATTEMPTED!\n");
        return 1;
    }
    _servos.controller->ledOffTime(SERVO_X_PIN, _servos.xValue);
    _servos.controller->ledOffTime(SERVO_Y_PIN, _servos.yValue);
    _servos.controller->ledOffTime(SERVO_Z_PIN, _servos.zValue);

    return 0;
}

// --- INVERSE KINEMATICS --- //

//
// Calculates the angle at the base of the arm (rotational)
//
double SSRobot::calculateGamma(double x, double y, double z){
    double g = atan2(x,y);
    return rads_to_degs(g);
}

//
// Calculates the angle at the shoulder of the arm
//
double SSRobot::calculateAlpha(double x, double y, double z){
    double a, a1, a2;
    double L1 = sqrt(pow(x,2) + pow(y,2));
    double L = sqrt(pow(z,2) + pow(L1,2));
    if(z < 0){
      z = std::abs(z);
      a1 = acos(z/L);
    }else{
      a1 = asin(z/L)+(PI/2);
    }
    a2 = acos((pow(TIBIA, 2) - pow(FEMUR,2) - pow(L,2)) / (-2*FEMUR*L));
    a = a1+a2;
    return rads_to_degs(a);
}

//
// Calculates the angle in the elbow part of the arm
//
double SSRobot::calculateBeta(double x, double y, double z){
    double L1 = sqrt(pow(x,2) + pow(y,2));
    double L = sqrt(pow(z,2) + pow(L1,2));
    double b = acos((pow(L,2) - pow(TIBIA,2) - pow(FEMUR,2)) / (-2*TIBIA*FEMUR));
    return rads_to_degs(b);
}

//
// Maps the current angle to PWM values for servos
//
pwms_t SSRobot::angleToPwm(double g, double a, double b){
    double pwm_x = interp(g,X_MIN_ANGLE,X_MAX_ANGLE,X_MIN_PWM,X_MAX_PWM);
    double pwm_y = interp(a,Y_MIN_ANGLE,Y_MAX_ANGLE,Y_MIN_PWM,Y_MAX_PWM);
    // Calculate current beta degrees due to mechanical structure
    int b2 = (180 - a) + BASE_ANGLE;
    // Calculate how much more beta axis needs to rotate where delta_b is degrees
    // servo needs to rotate.
    double delta_b = b2 - b;
    double bonusAngle = interp(pwm_y,Y_MIN_PWM,Y_MAX_PWM,40,0);
    double bonusPwm = interp(pwm_y,Y_MIN_PWM,Y_MAX_PWM,350,0);
    // map servo_z rotation
    double pwm_z = interp(delta_b,Z_MIN_ANGLE,Z_MAX_ANGLE+bonusAngle,Z_MIN_PWM,Z_MAX_PWM+bonusPwm);

    // Scale PWM values for Adafruit PWM Controller
    double scalar = (((1.0 / PWM_HZ)*1000000.0) / PWM_RESOLUTION);
    pwm_x = pwm_x / scalar;
    pwm_y = pwm_y / scalar;
    pwm_z = pwm_z / scalar;

    pwms_t retPos = {(int)pwm_x, (int)pwm_y, (int)pwm_z};

    return retPos; 
}