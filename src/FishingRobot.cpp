/*
* @Author: Bryant Hayes
* @Date:   2016-05-14 19:02:35
* @Last Modified by:   bryanthayes
* @Last Modified time: 2016-05-25 01:51:01
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
	setPosition(0,35,-15);
    _armSpeed = 100;
    setPosition(-25,25,-10);
    //usleep(500000);
    setPosition(_lastXYZ.x,_lastXYZ.y,-23.4);
    usleep(1100000);
    _armSpeed = lastSpeed;
    setPosition(_targetXYZ.x,_targetXYZ.y,-20.5);
    //usleep(1000000);
}

//
// Goes down and then back up to attempt to grab a fish
//
void FishingRobot::grab() {
	int lastSpeed = _armSpeed;
	_armSpeed = 99;
	setPosition(_targetXYZ.x, _targetXYZ.y, -24, 320000);
	//usleep(1500000-_armSpeed*10000);
	_armSpeed = lastSpeed;
	//usleep(300000);
	setPosition(_lastXYZ.x, _lastXYZ.y, _lastXYZ.z);
	//usleep(1000000);
}

//
// Goes down and then back up to attempt to grab a fish
//
void FishingRobot::grab(int idx) {
	int lastSpeed = _armSpeed;
	_armSpeed = 99;
	if(idx == 4 || idx == 5) {
		setPosition(_targetXYZ.x, _targetXYZ.y, -24, 1000000);
	} else {
		setPosition(_targetXYZ.x, _targetXYZ.y, -24, 400000);
	}
	//usleep(1500000-_armSpeed*10000);
	_armSpeed = lastSpeed;
	//usleep(300000);
	setPosition(_lastXYZ.x, _lastXYZ.y, _lastXYZ.z);
	//usleep(1000000);
}

void FishingRobot::shake(){
	int lastSpeed = _armSpeed;
	double shake_x = -25.0;
	double shake_y = 20.0;
	double shake_z_up = -14.0;
	double shake_z_down = -23;
	double shake_amount = 1.5;
	double previous_x = _targetXYZ.x;
	double previous_y = _targetXYZ.y;
	double previous_z = _targetXYZ.z;

	_armSpeed = 75;
	setPosition(previous_x,previous_y,shake_z_up);
	_armSpeed = 90;
	usleep(20000);
	setPosition(shake_x,shake_y,shake_z_up,1250000);
	setPosition(shake_x,shake_y,shake_z_down,1000000);
	_armSpeed = 100;
	for(int i = 0; i < 2; i++){
		setPosition(shake_x-shake_amount,shake_y-shake_amount,shake_z_down);
		setPosition(shake_x+shake_amount,shake_y+shake_amount,shake_z_down);
	}
	_armSpeed = 99;
	usleep(20000);
	setPosition(shake_x,shake_y,shake_z_up);
	setPosition(previous_x,previous_y,previous_z);

	_armSpeed = lastSpeed;
	usleep(20000);
}

void FishingRobot::setFishPosition() {
	setPosition(-13.5,30,-16.25,3000000);
	return;
}


