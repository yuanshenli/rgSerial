
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "sb_teensy_pin_m8.h"

/*
 * driver_channel_separate		serial_channel		physical_joint		ctrl_channel    tele_op_gui_ctrl
			0					        0				front base				0               0
			1					        1				left base				3               1
			2					        2				right base				6               2
			3					        3				palm base				9               3
			0					        4				left pivot				4               5
			1					        5				left roller				5               9
			2					        6				front pivot				1               4
			3					        7				front roller			2               8
			4					        8				palm pivot				10              7
			5					        9				palm roller				11              11
			6					        10				right pivot				7               6
			7					        11				right roller			8               10
 * */

const int pwmPinN[8] = {18, 14, 11, 22, 7, 9, 4, 2};
const int pwmPinP[8] = {19, 15, 10, 12, 6, 8, 5, 3};
//const int enablePin[8] = {18, 19, 20, 21, 22, 23};

const int ENC_A[8] = {33, 35, 37, 39, 31, 29, 24, 26};
const int ENC_B[8] = {34, 36, 38, 32, 30, 28, 25, 27};

float Setpoint[8], Input[8], Output[8];
float SetpointPos[8], InputPos[8], OutputPos[8];
volatile int lastEncoded[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile long encoderValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile long lastEncoderValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int lastMSB[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int lastLSB[8] = {0, 0, 0, 0, 0, 0, 0, 0};


void updateEncoder0(){
    int MSB = digitalRead(ENC_A[0]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[0]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[0] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[0] --;

    lastEncoded[0] = encoded; //store this value for next time
    InputPos[0] = encoderValue[0];
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
    InputPos[1] = encoderValue[1];
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
    InputPos[2] = encoderValue[2];
}

void updateEncoder3(){
    int MSB = digitalRead(ENC_A[3]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[3]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[3] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[3] --;

    lastEncoded[3] = encoded; //store this value for next time
    InputPos[3] = encoderValue[3];
//     Serial.println(encoderValue[3]);
}


void updateEncoder4(){
    int MSB = digitalRead(ENC_A[4]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[4]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[4] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[4] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[4] --;

    lastEncoded[4] = encoded; //store this value for next time
    InputPos[4] = encoderValue[4];
}

void updateEncoder5(){
    int MSB = digitalRead(ENC_A[5]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[5]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[5] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[5] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[5] --;

    lastEncoded[5] = encoded; //store this value for next time
    InputPos[5] = encoderValue[5];
}

void updateEncoder6(){
    int MSB = digitalRead(ENC_A[6]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[6]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[6] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[6] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[6] --;

    lastEncoded[6] = encoded; //store this value for next time
    InputPos[6] = encoderValue[6];
}

void updateEncoder7(){
    int MSB = digitalRead(ENC_A[7]); //MSB = most significant bit
    int LSB = digitalRead(ENC_B[7]); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded[7] << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[7] ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[7] --;

    lastEncoded[7] = encoded; //store this value for next time
    InputPos[7] = encoderValue[7];
}
