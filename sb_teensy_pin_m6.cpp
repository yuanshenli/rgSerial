
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sb_teensy_pin_m6.h"



const int pwmPinN[6] = {2, 4, 6, 8, 35, 37};
const int pwmPinP[6] = {3, 5, 7, 9, 36, 38};
const int enablePin[6] = {18, 19, 20, 21, 22, 23};
// const int ENC_A[6] = {24, 26, 28, 30, 32, 34}; 
// const int ENC_B[6] = {25, 27, 29, 31, 33, 39};
const int ENC_A[6] = {10, 12, 25, 27, 29, 31};
const int ENC_B[6] = {11, 24, 26, 28, 30, 32}; 





float Setpoint[6], Input[6], Output[6];
volatile int lastEncoded[6] = {0, 0, 0, 0, 0, 0};
volatile long encoderValue[6] = {0, 0, 0, 0, 0, 0};
long lastencoderValue[6] = {0, 0, 0, 0, 0, 0};
int lastMSB[6] = {0, 0, 0, 0, 0, 0};
int lastLSB[6] = {0, 0, 0, 0, 0, 0};


void updateEncoder0(){
  int MSB = digitalRead(ENC_A[0]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[0]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[0] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[0] --;

  lastEncoded[0] = encoded; //store this value for next time
  Input[0] = encoderValue[0];
  // Serial1.println(encoderValue[0]);
}

void updateEncoder1(){
  int MSB = digitalRead(ENC_A[1]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[1]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[1] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[1] --;

  lastEncoded[1] = encoded; //store this value for next time
  Input[1] = encoderValue[1];
  // Serial.println(encoderValue[1]);
}

void updateEncoder2(){
  int MSB = digitalRead(ENC_A[2]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[2]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[2] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[2] --;

  lastEncoded[2] = encoded; //store this value for next time
  Input[2] = encoderValue[2];
}

void updateEncoder3(){
  int MSB = digitalRead(ENC_A[3]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[3]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[3] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[3] --;

  lastEncoded[3] = encoded; //store this value for next time
  Input[3] = encoderValue[3];
}


void updateEncoder4(){
  int MSB = digitalRead(ENC_A[4]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[4]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[4] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[4] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[4] --;

  lastEncoded[4] = encoded; //store this value for next time
  Input[4] = encoderValue[4];
}

void updateEncoder5(){
  int MSB = digitalRead(ENC_A[5]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[5]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[5] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[5] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[5] --;

  lastEncoded[5] = encoded; //store this value for next time
  Input[5] = encoderValue[5];
}