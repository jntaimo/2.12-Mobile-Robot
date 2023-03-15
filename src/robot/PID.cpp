#include <Arduino.h>
#include "encoder.h"

void setup(){
    Serial.begin(115200);
    encoderSetup();
    
}

void loop(){
    readEncoders();
    Serial.println(encFLRad);
    delay(10);
}


//returns the command signal for an academic PID loop
//sum_error will be update to maintain the cumulative error sum
//this requires tracking of variables outside of the PID function
float runPID(float error,float last_error, float kp, float ki, float kd, float &sum_error, float max_error, float loop_time){
    sum_error += ki*error*loop_time;
    sum_error = constrain(sum_error, -max_error, max_error);
    float P = kp*error;
    float I = ki*sum_error;
    float D = kd*(error-last_error)/loop_time;
    return P + I + D;
}