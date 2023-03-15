#include <Arduino.h>
#include "encoder.h"


//filtered velocity of each wheel in radians
float filtVelFL = 0;
float filtVelBL = 0;
float filtVelFR = 0;
float filtVelBR = 0;

//scaling factor for each new reading
//if alpha = 0, each new reading is not even considered
//if alpha = 1, each new reading is the only thing considered
//lower values of alpha smooth the filtered velocity more, but delay the signal
float alpha = 0.1;

unsigned long prevPIDTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long pidDelayMicros = 5000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 100;


void setup(){
    Serial.begin(115200);
    encoderSetup();
}

void loop(){
    if (micros() - prevPIDTimeMicros > pidDelayMicros){
        prevPIDTimeMicros = micros();
        readEncoders(); 
    }
    
    if (millis() - prevPrintTimeMillis > printDelayMillis){
        prevPrintTimeMillis = millis();
        Serial.println(encFLRad);
    }
}

//updates the filtered velocity values
//should be run as frequently as possible
void updateVelocity(){

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