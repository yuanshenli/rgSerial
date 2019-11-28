#ifndef SB_TEENSY_PIN_H
#define SB_TEENSY_PIN_H

extern const int pwmPinN[6];
extern const int pwmPinP[6];
extern const int ENC_A[6];
extern const int ENC_B[6];
extern const int enablePin[6];


extern float Setpoint[6], lastSetpoint[6], Input[6], Output[6];
extern volatile int lastEncoded[6];
extern volatile long encoderValue[6];
extern long lastencoderValue[6];
extern int lastMSB[6];
extern int lastLSB[6];

void updateEncoder0();
void updateEncoder1();
void updateEncoder2();
void updateEncoder3();
void updateEncoder4();
void updateEncoder5();

#endif