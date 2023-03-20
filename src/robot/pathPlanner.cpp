#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"



//TODO wheel radius in meters
#define r 0.06
//TODO wheel distance
#define b 0.5

float pathDistance = 0;
//x and y position of the robot in meters
float x = 0;
float y = 0;
float theta = 0;

float dPhiFL = 0;
float dPhiBL = 0;
float dPhiFR = 0;
float dPhiBR = 0;

//allows the intergral control to max contribution at the max drive voltage
//prevents integral windum
float maxSumError = (DRIVE_VOLTAGE/ki)/2;

unsigned long prevLoopTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

void setDesiredVel(float vel, float k);
void updateRobotPose(float dPhiL, float dPhiR);
void getSetPointTrajectory();
void printOdometry();
void sendOdometry();

void setup(){
    encoderSetup();
    driveSetup();
    wirelessSetup();
}

void loop(){
    if (micros() - prevLoopTimeMicros > loopDelayMicros){
        prevLoopTimeMicros = micros();
        //get new encoder readings and update the velocity
        //also updates dPhi values for the change in angle of each motor
        updateVelocity(loopDelayMicros*1e-6);

        //dRad is the change in radians since the last reading of the encoders
        //just use the back left and back right encoders to calculate trajectory
        updateRobotPose(dPhiBL, dPhiBR);

        //print out the values over the serial port
        printOdometry();

        //sends odometry to the remote
        sendOdometry();

        //uncomment the desired method for updating the PI setpoint

        //getSetPointTrajectory();
        //getSetPointDriveTest();
        //getSetPointJoystick();

        //calculate error for each motor
        float newErrorFL = desiredVelFL - filtVelFL;
        float newErrorBL = desiredVelBL - filtVelBL;
        float newErrorFR = desiredVelFR - filtVelFR;
        float newErrorBR = desiredVelBR - filtVelBR;

        //get control signal by running PID on all for motors
        voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros*1e-6);      
        voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros*1e-6);
        voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros*1e-6);            
        voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros*1e-6);
        
        //only drive the back motors
        driveVolts(0, voltageBL, 0, voltageBR);
    }
    

    //put print statements here
    if (millis() - prevPrintTimeMillis > printDelayMillis){
        prevPrintTimeMillis = millis();

        //print Back left wheel data for debugging
        //Serial.printf("v: %f filtvel: %f desiredvel: %f sumerror: %f\n", voltageBL, filtVelBL, desiredVelBL, sumErrorBL);
        //Serial.println(filtVelFL);
        //uncomment to print current joystick readings
        //Serial.printf("JoyX: %d JoyY %d\n", joyData.joyX, joyData.joyY);
    }

}

//sets the desired velocity based on desired velocity vel in m/
//and k curvature in 1/m representing 1/(radius of curvature)
void setDesiredVel(float vel, float k){
    //TODO convert the velocity and k curvature to new values for desiredVelBL and desiredVelBR
    desiredVelBL = vel - k*b*vel;
    desiredVelFL = desiredVelBL;
    desiredVelBR = 2*vel - desiredVelBL;
    desiredVelFR = desiredVelBR;
}

//makes robot follow a trajectory
void getSetPointTrajectory(){
    //default to not moving
    //velocity in m/s
    //k is 1/radius from center of rotation circle
    float vel = 0 , k = 0;
    //TODO Add trajectory planning by changing the value of vel and k
    //based on odemetry conditions
    if (pathDistance <= 1.0){
        //STRAIGHT LINE FORWARD
        vel = 0.2;
        k = 0;
    } else if (pathDistance > 1 && pathDistance < (1+0.25*PI)){
        //TURN IN SEMICIRCLE
        vel = 0.2;
        k = 1/0.25;
    } else if (pathDistance > (1+ 0.25*PI) && pathDistance < (2 + 0.25*PI)){
        //STRAIGHT LINE BACK
        vel = 0.2;
        k = 0;
    } else {
        //STOP
        vel = 0;
        k = 0;
    }
    setDesiredVel(vel, k);
}

//updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR){
    //TODO change in angle
    float dtheta = r/(2*b)*(dPhiR-dPhiL);
    //TODO update theta value
    theta += dtheta;
    //TODO use the equations from the handout to calculate the change in x and y
    float dx = r/2 * (cos(theta)*dPhiR + cos(theta)*dPhiL);
    float dy = r/2 * (sin(theta)*dPhiR + sin(theta)*dPhiL);
    //TODO update x and y positions
    x += dx;
    y += dy;
    //TODO update the pathDistance
    pathDistance += sqrt(dx*dx + dy*dy);
}

//prints currecnt odometry to be read into MATLAB
void printOdometry(){

}

//sends odometry data to the remote
void sendOdometry(){

}