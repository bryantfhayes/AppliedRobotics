/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 13:21:56
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-14 13:21:56
*/

#ifndef __SSROBOT_H
#define __SSROBOT_H

#include "mraa.hpp"
#include "pca9685.h"

#define NUM_SERVOS 3
#define SERVO_ENABLE_PIN 6
#define SERVO_X_PIN 0
#define SERVO_Y_PIN 1
#define SERVO_Z_PIN 2
#define DEFAULT_PWM 1245
#define PWM_HZ 200
#define PWM_RESOLUTION 4096

// Inverse Kinematic Constants
#define TOOL_LENGTH 22
#define TOOL_HEIGHT 16
#define X_MAX_PWM  1037
#define X_MIN_PWM  2136
#define Y_MAX_PWM  1678
#define Y_MIN_PWM  915
#define Z_MAX_PWM  1586
#define Z_MIN_PWM  915
#define X_MAX_ANGLE  57
#define X_MIN_ANGLE  -63
#define Y_MAX_ANGLE  185
#define Y_MIN_ANGLE  104
#define Z_MAX_ANGLE  65
#define Z_MIN_ANGLE  0
#define BASE_ANGLE 90
#define TIBIA 15.0
#define FEMUR 14.0

// Miscellaneous
#define PI 3.141592653

// Servo safety bounds
#define SERVO_X_MIN 830
#define SERVO_X_MAX 1770
#define SERVO_Y_MIN 730
#define SERVO_Y_MAX 1395
#define SERVO_Z_MIN 730
#define SERVO_Z_MAX 1620

// Inline Functions
// inline int max ( int a, int b ) { return a > b ? a : b; }

typedef struct servos_t {
	upm::PCA9685* controller;
	int xValue, yValue, zValue;
	mraa::Gpio* enablePin;
} servos_t;

typedef struct position_t {
	double x,y,z;
} position_t;

typedef struct pwms_t {
	int x,y,z;
	bool operator == (const pwms_t &rhs){
		return (this->x == rhs.x && this->y == rhs.y && this->z == rhs.z);
	}
} pwms_t;

class SSRobot {
	static void* update(void* context);
    public:
        SSRobot();
        ~SSRobot();
        int setPosition(double x, double y, double z);
        int setPosition(double x, double y, double z, int delay);
        void setSpeed(int x);
        int getSpeed();
        void enable(bool state);
        void* startUpdateThread(void* context);
        bool isAlive();
    protected:
    	pwms_t angleToPwm(double g, double a, double b);
    	double calculateBeta(double x, double y, double z);
    	double calculateGamma(double x, double y, double z);
    	double calculateAlpha(double x, double y, double z);
    	int initServos();
    	int initControlThread();
    	int updateServos();
        int biggestDifference(pwms_t a, pwms_t b);
    	servos_t _servos;
    	pwms_t _targetPWM;
    	position_t _targetXYZ;
    	pwms_t _currentPWM;
    	position_t _lastXYZ;
    	pthread_t _controlThread;
    	int _armSpeed;
    	int _rPtr;
    	bool _alive;
};

#endif