#ifndef SB_TEENSY_PIN_H
#define SB_TEENSY_PIN_H

extern const int pwmPinN[8];
extern const int pwmPinP[8];
extern const int ENC_A[8];
extern const int ENC_B[8];
extern const int enablePin[8];


extern float Setpoint[8], lastSetpoint[8], Input[8], Output[8];
extern float SetpointPos[8], InputPos[8], OutputPos[8];
extern volatile int lastEncoded[8];
extern volatile long encoderValue[8];
extern volatile long lastEncoderValue[8];
extern int lastMSB[8];
extern int lastLSB[8];

void updateEncoder0();
void updateEncoder1();
void updateEncoder2();
void updateEncoder3();
void updateEncoder4();
void updateEncoder5();
void updateEncoder6();
void updateEncoder7();

#endif