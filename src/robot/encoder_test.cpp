#include <Arduino.h>
#include <SPI.h>
#include <Encoder_Buffer.h>


//Chip select pins
//In order to read each encoder separately, one of these
//pins is set to high at a time
#define CS1 4
#define CS2 16
#define CS3 17
#define CS4 21


//Pulses per revolution of the motor
//Differs based on gear ratio
//For the 312 RPM Motor 
#define MOTOR_PPR 537.7
//The encoder reads in 4x mode, so it gets quadruple resolution
#define ENC_PPR MOTOR_PPR*4

//Diameter is 120mm converted to meters
#define WHEEL_RADIUS_M 0.06

//Encoder Objects for each motor
Encoder_Buffer EncoderFL(CS1);
Encoder_Buffer EncoderBL(CS2);
Encoder_Buffer EncoderFR(CS3); 
Encoder_Buffer EncoderBR(CS4);

//Encoder Counts
long encFLCount = 0;
long encBLCount = 0;
long encFRCount = 0;
long encBRCount = 0;

void printEncoderCounts();
void clearEncoders();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Starts communication protocol to communicate with the encoder breakout
  SPI.begin();
  //Initialize all the encoders
  EncoderFL.initEncoder();
  EncoderBL.initEncoder();
  EncoderFR.initEncoder();
  EncoderBR.initEncoder();
}
//Delay (in milliseconds) between consecutive reads of the encoders
//Prevents constantly
unsigned long encoderReadDelay = 200;
unsigned long lastEncoderRead = 0;

void loop() {
  //If we have waited long enough since the last encoder read
  if (millis() - lastEncoderRead > encoderReadDelay){
  //read all the encoders
  //note that the front right and back right encoders are flipped
  //this is to maintain movement forward as positive for both sides
  encFLCount = EncoderFL.readEncoder();
  encBLCount = EncoderBL.readEncoder();
  encFRCount = -EncoderFR.readEncoder();
  encBRCount = -EncoderBR.readEncoder();
  
  printEncoderCounts();
  //printEncoderRadians();
  //printEncoderDistance();

  //reset the encoder time delay
  lastEncoderRead = millis();
  }

}




//Prints current encoder counts
void printEncoderCounts(){
  //Prints out the encoder counts
  //printf lets you insert numbers into strings
  //the '%d' is replaced by the encoder count to the right of the comma
  //See here for more info
  //https://www.programiz.com/cpp-programming/library-function/cstdio/printf
  Serial.printf("FL Count: %d\n", encFLCount);
  Serial.printf("BL Count: %d\n", encBLCount);
  Serial.printf("FR Count: %d\n", encFRCount);
  Serial.printf("BR Count: %d\n", encBRCount);
}


//prints the current encoder rotations in Radians
void printEncoderRadians(){
  //Converts the encoder counts to radians
  //Count/PPR gives number of rotations
  //multiply by 2pi since there are 2pi radians per rotation
  Serial.printf("FL Radians: %f\n", encFLCount*2*PI/ENC_PPR);
  Serial.printf("BL Radians: %f\n", encBLCount*2*PI/ENC_PPR);
  Serial.printf("FR Radians: %f\n", encFRCount*2*PI/ENC_PPR);
  Serial.printf("BR Radians: %f\n", encBRCount*2*PI/ENC_PPR);

}

//Prints the distance the robot traverses using the stock wheels
void printEncoderDistance(){
  //S = R*theta
  Serial.printf("FL Radians: %f\n", encFLCount*2*PI*WHEEL_RADIUS_M/ENC_PPR);
  Serial.printf("BL Radians: %f\n", encBLCount*2*PI*WHEEL_RADIUS_M/ENC_PPR);
  Serial.printf("FR Radians: %f\n", encFRCount*2*PI*WHEEL_RADIUS_M/ENC_PPR);
  Serial.printf("BR Radians: %f\n", encBRCount*2*PI*WHEEL_RADIUS_M/ENC_PPR);

}

//Clears all of the encoder values 
void clearEncoders(){
  //Encoder Counts
  encFLCount = 0;
  encBLCount = 0;
  encFRCount = 0;
  encBRCount = 0;

  EncoderFL.clearEncoderCount();
  EncoderBL.clearEncoderCount();
  EncoderFR.clearEncoderCount();
  EncoderBR.clearEncoderCount();
}
