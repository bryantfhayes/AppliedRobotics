#ifndef __IKHELPER_H
#define __IKHELPER_H

#define NUM_SERVOS 3

#define X_MAX_PWM  610
#define X_MIN_PWM  2210
#define Y_MAX_PWM  1700
#define Y_MIN_PWM  1000
#define Z_MAX_PWM  1600
#define Z_MIN_PWM  900
#define X_MAX_ANGLE  90
#define X_MIN_ANGLE  -90
#define Y_MAX_ANGLE  180
#define Y_MIN_ANGLE  100
#define Z_MAX_ANGLE  70
#define Z_MIN_ANGLE  0
#define BASE_ANGLE 90

#define TIBIA 15.5
#define FEMUR 14.0

#define PI 3.141592653

void angle_to_pwm(double g, double a, double b, int pwm_values[]);

int interp(double n, int x1, int x2, int y1, int y2);

double rads_to_degs(double x);

double get_gamma(double x, double y, double z);

double get_alpha(double x, double y, double z);

double get_beta(double x, double y, double z);

void scale_pwm(int pwm_values[]);

int XYZ_to_PWM(double x, double y, double z, int pwm_values[]);

#endif
