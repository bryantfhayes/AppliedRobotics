/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 19:02:35
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-18 21:50:22
*/

#include <iostream>

#include "FishingRobot.h"

using namespace std;

//
// Constructor
//
FishingRobot::FishingRobot(){
	boardAngle = 0.0;
}

//
// Destructor
//
FishingRobot::~FishingRobot(){
}

//
// Tosses a fish sideways very quickly
//
void FishingRobot::toss() {
	int lastSpeed = _armSpeed;
    _armSpeed = 100;
    setPosition(-25,25,-10);
    usleep(500000);
    setPosition(_lastXYZ.x,_lastXYZ.y,-23);
    usleep(1500000);
    _armSpeed = lastSpeed;
    setPosition(_targetXYZ.x,_targetXYZ.y,-20.5);
    usleep(1000000);
}

//
// Goes down and then back up to attempt to grab a fish
//
void FishingRobot::grab() {
	int lastSpeed = _armSpeed;
	_armSpeed = 99;
	setPosition(_targetXYZ.x, _targetXYZ.y, -24);
	usleep(1500000-_armSpeed*10000);
	_armSpeed = lastSpeed;
	usleep(300000);
	setPosition(_lastXYZ.x, _lastXYZ.y, _lastXYZ.z);
	usleep(1000000);
}

void FishingRobot::shake(){
	int lastSpeed = _armSpeed;
	_armSpeed = 50;
	double x = _targetXYZ.x;
	double y = _targetXYZ.y;
	double z = _targetXYZ.z;
	setPosition(x,y,-14);
	usleep(1200000);
	_armSpeed = 80;
	usleep(20000);
	setPosition(-16.5,30,-14);
	usleep(1200000);
	setPosition(-16.5,30,-21);
	usleep(1000000);
	_armSpeed = 100;
	for(int i = 0; i < 2; i++){
		setPosition(-15,30,-21);
		usleep(250000);
		setPosition(-18,30,-21);
		usleep(250000);
	}
	setPosition(-16.5,30,-15);
	usleep(1000000);
	_armSpeed = 100;
	usleep(50000);
	setPosition(x,y,z);
	usleep(400000);
	_armSpeed = lastSpeed;
	usleep(50000);
}


