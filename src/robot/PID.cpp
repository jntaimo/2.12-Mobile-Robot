#include <Arduino.h>
#include "encoder.h"

//instantaneous velocity of each wheel in radians per second
float velFL = 0;
float velBL = 0;
float velFR = 0;
float velBR = 0;

//filtered velocity of each wheel in radians per second
float filtVelFL = 0;
float filtVelBL = 0;
float filtVelFR = 0;
float filtVelBR = 0;


//scaling factor for each new reading
//if alpha = 0, each new reading is not even considered
//if alpha = 1, each new reading is the only thing considered
//lower values of alpha smooth the filtered velocity more, but delay the signal
float alpha = 0.05;

unsigned long prevPIDTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long pidDelayMicros = 10000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

void updateVelocity();

void setup(){
    Serial.begin(115200);
    encoderSetup();
}

void loop(){
    if (micros() - prevPIDTimeMicros > pidDelayMicros){
        prevPIDTimeMicros = micros();
        updateVelocity();
    }
    
    if (millis() - prevPrintTimeMillis > printDelayMillis){
        prevPrintTimeMillis = millis();
        Serial.println(filtVelFL);
    }
}

//updates the filtered velocity values
//should be run every pidDelayMicros microseconds
void updateVelocity(){
    //store current positions to reference
    float lastRadFL = encFLRad;
    float lastRadBL = encBLRad;
    float lastRadFR = encFRRad;
    float lastRadBR = encBRRad;
    //get new positions
    readEncoders();
    //convert time from microseconds to seconds
    float dt = pidDelayMicros*1e-6;
    //get (change in position)/time
    velFL = (encFLRad - lastRadFL)/dt;
    velBL = (encBLRad - lastRadBL)/dt;
    velFR = (encFRRad - lastRadFR)/dt;
    velBR = (encBRRad - lastRadBR)/dt;
    //use first order alpha based filter to get filtered velocities
    filtVelFL = alpha*velFL + (1-alpha)*filtVelFR;
    filtVelBL = alpha*velBL + (1-alpha)*filtVelBL;
    filtVelFR = alpha*velFR + (1-alpha)*filtVelFR;
    filtVelBR = alpha*velBR + (1-alpha)*filtVelBR;
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