#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "receiver.h"
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

//sum errors for integral term
float sumErrorFL = 0;
float sumErrorBL = 0;
float sumErrorFR = 0;
float sumErrorBR = 0;


//desired velocity setpoints in rad/s
float desiredVelFL = 0;
float desiredVelBL = 0;
float desiredVelFR = 0;
float desiredVelBR = 0;

//voltage to send to the motors
float voltageFL = 0;
float voltageBL = 0;
float voltageFR = 0;
float voltageBR = 0;

//error readings
float errorFL = 0;
float errorBL = 0;
float errorFR = 0;
float errorBR = 0;

//PID Constants
float kp = 20;
float ki = 20;
float kd = 0;

//allows the intergral control to max contribution at the max drive voltage
//prevents integral windum
float maxSumError = DRIVE_VOLTAGE/ki;


unsigned long prevPIDTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long pidDelayMicros = 10000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

//function prototypes
void updateVelocity();
float runPID(float error,float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime);

void setup(){
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    desiredVelBL = 1;
    desiredVelBR = 1;
}


void loop(){
    // if (micros() - prevPIDTimeMicros > pidDelayMicros){
    //     prevPIDTimeMicros = micros();
    //     updateVelocity();

    //     float newErrorFL = desiredVelFL - filtVelFL;
    //     float newErrorBL = desiredVelBL - filtVelBL;
    //     float newErrorFR = desiredVelFR - filtVelFR;
    //     float newErrorBR = desiredVelBR - filtVelBR;
        
    //     voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, pidDelayMicros*1e-6);
    //     voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, pidDelayMicros*1e-6);
    //     driveVolts(0, 5, 0, 0);
    // }
    
    // if (millis() - prevPrintTimeMillis > printDelayMillis){
    //     prevPrintTimeMillis = millis();
    //     Serial.printf("%f\t%f\t%f\t%f\n", voltageBL, filtVelBL, desiredVelBL, sumErrorBL);

    // }
    Serial.println("Front Left");
    driveVolts(5, 0,0,0);
    delay(4000);
    Serial.println("Front Right");
    driveVolts(0, 0,5,0);
    delay(4000);
    Serial.println("Back Left");
    driveVolts(0, 5,0,0);
    delay(4000);
    Serial.println("Back Right");
    driveVolts(0, 0,0,5);
    delay(4000);

    
}


//updates the wheel setpoint based on the current joystick values
void getSetPointJoystick(){


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
float runPID(float error,float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime){
    sumError += error*loopTime;
    //avoid integral windum
    sumError = constrain(sumError, -maxSumError, maxSumError);
    //standard PID configuration
    float P = kp*error;
    float I = ki*sumError;
    float D = kd*(error-last_error)/loopTime;
    return P + I + D;
}