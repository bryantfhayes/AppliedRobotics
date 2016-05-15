/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 19:02:35
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-14 20:59:28
*/

#include <iostream>

#include "FishingRobot.h"

using namespace std;

//
// Constructor
//
FishingRobot::FishingRobot(){
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
    setPosition(-30,20,-10);
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


