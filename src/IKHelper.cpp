#include "IKHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

static double theta = 0.0;

void angle_to_pwm(double g, double a, double b, int pwm_values[]){
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
    // Save and return
    pwm_values[0] = pwm_x;
    pwm_values[1] = pwm_y;
    pwm_values[2] = pwm_z;

    //for(int i = 0; i < NUM_SERVOS; i++){
    //   fprintf(stdout, "raw_pwm %d = %d\n", i, pwm_values[i]);
    //}
    fprintf(stdout, "angles: g: %lf, a: %lf, b: %lf\n", g, a, b);

    return; 
}

int interp(double n, int x1, int x2, int y1, int y2){
    double slope = 1.0 * (y2 - y1) / (x2 - x1);
    int output = y1 + round(slope * (n - x1));
    if((y2 > y1) && (output > y2)) return y2;
    if((y2 > y1) && (output < y1)) return y1;
    if((y2 < y1) && (output > y1)) return y1;
    if((y2 < y1) && (output < y2)) return y2;
    return output;
}

double rads_to_degs(double x){
    return x*(180/PI);
}

double get_gamma(double x, double y, double z){
    double g = atan2(x,y);
    theta = g;
    return rads_to_degs(g);
}

double get_alpha(double x, double y, double z){
    double a, a1, a2;
    double L1 = sqrt(pow(x,2) + pow(y,2));
    double L = sqrt(pow(z,2) + pow(L1,2));
    if(z < 0){
      z = abs(z);
      a1 = acos(z/L);
    }else{
      a1 = asin(z/L)+(PI/2);
    }
    a2 = acos((pow(TIBIA, 2) - pow(FEMUR,2) - pow(L,2)) / (-2*FEMUR*L));
    a = a1+a2;
    return rads_to_degs(a);
}

double get_beta(double x, double y, double z){
    double L1 = sqrt(pow(x,2) + pow(y,2));
    double L = sqrt(pow(z,2) + pow(L1,2));
    double b = acos((pow(L,2) - pow(TIBIA,2) - pow(FEMUR,2)) / (-2*TIBIA*FEMUR));
    return rads_to_degs(b);
}

void scale_pwm(int pwm_values[]){
    for(int i = 0; i < NUM_SERVOS; i++){
        pwm_values[i] = interp(pwm_values[i], 900, 2100, 736, 1717);
    }
}

int XYZ_to_PWM(double x, double y, double z, int pwm_values[]){
    // Determine required joint angles
    double g = get_gamma(x,y,z);
    double a = get_alpha(x-(22*sin(theta)),y-(22*cos(theta)),z+16);
    double b = get_beta(x-(22*sin(theta)),y-(22*cos(theta)),z+16);
    //double a = get_alpha(x,y,z+16);
    //double b = get_beta(x,y,z+16);
    
    if(isnan(a) || isnan(b) || isnan(g)){
        return -1;
    }
    // Determine PWM values
    angle_to_pwm(g, a, b, pwm_values);
    // map pwm values to range required for 1/4096 resolution
    scale_pwm(pwm_values);

    return 0;
}

