#include <Arduino.h>
#include "speed_motor.h"
#include "sb_motor.h"
#include "sb_message.h"
#include <algorithm>


#define DEBUG_SERIAL Serial4
#define ANALOG_RES 12
#define PWM_FREQ 20000
#define BIT2DEG  360.0/pow(2.0, (float)ANALOG_RES)
#define DEG2BIT  pow(2.0, (float)ANALOG_RES)/360.0

/* Communication */
const byte numBytes = 64;
byte receiveBuffer[numBytes];
byte sendBuffer[numBytes];

uint8_t incomingLength = 0;

enum recState {
    readHeaderByte,
    readLengthByte,
    readDataByte,
    readError
};
recState curState = readHeaderByte;

byte headerByte = 0x7E;
uint8_t idx = 0;
byte rc;

SBRegisters regAddr;

enum controlMode {
    pos_ctrl = 0,
    vel_ctrl = 1
};

controlMode motorCtrlMode[8] = {pos_ctrl, pos_ctrl, pos_ctrl, pos_ctrl, pos_ctrl, pos_ctrl, pos_ctrl, pos_ctrl};

/* DC Motors*/
#define CPR 3040.7596
#define K_P 15.0
#define K_I 0.2
#define K_D 100.0

Speed_Motor motor[8] = {
        Speed_Motor(0, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(1, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(2, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(3, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(4, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(5, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(6, CPR, K_P, K_I, K_D, Input, Output, Setpoint),
        Speed_Motor(7, CPR, K_P, K_I, K_D, Input, Output, Setpoint)
};

#define K_P_POS 14.0
#define K_I_POS 0
#define K_D_POS 0.3


SB_Motor motorPos[8] = {
        SB_Motor(0, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(1, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(2, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(3, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(4, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(5, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(6, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos),
        SB_Motor(7, CPR, K_P_POS, K_I_POS, K_D_POS, InputPos, OutputPos, SetpointPos)
};

int activeMotorIdx[4] = {0, 1, 2, 3};
int nMotorActivated = 0;


uint32_t startTime = millis();
int myCounter = 0;

void sendToSerial();

void recvFromSerial();

void processReceiveBuffer();

void printReceiveBuffer();

void printSendBuffer(uint8_t);

void processMotors(uint16_t motorID, uint16_t relativeMotorAddr, char *data, uint8_t recLength, SBTypes recType);

void printError(uint16_t relativeMotorAddr, uint16_t motorID);

void printState(uint16_t motorID, uint16_t val, const String &state);

void blinkNTimes(uint8_t num);

void setup() {
    Serial.begin(1000000);
    DEBUG_SERIAL.begin(115200);
    analogReadResolution(ANALOG_RES);
    for (int i = 0; i < 8; i++) {
        analogWriteFrequency(pwmPinN[i], PWM_FREQ);
        analogWriteFrequency(pwmPinP[i], PWM_FREQ);
    }


    delay(2000);
    pinMode(LED_BUILTIN, OUTPUT);
    blinkNTimes(5);
}

void loop() {
    recvFromSerial();
//    printReceiveBuffer();
    for (int i; i < 4;  i++) {
        if (motorCtrlMode[activeMotorIdx[i]] == pos_ctrl) {
            if (motorPos[activeMotorIdx[i]].GetInit() && !motorPos[activeMotorIdx[i]].GetStop()) {
                motorPos[activeMotorIdx[i]].Update();
            }
        } else if (motorCtrlMode[activeMotorIdx[i]] == vel_ctrl) {
            if (motor[activeMotorIdx[i]].GetInit() && !motor[activeMotorIdx[i]].GetStop()) {
                motor[activeMotorIdx[i]].Update();
            }
        }
    }
}


/* Modified version to accommodate the additional finger */
void sendToSerial() {
    sendBuffer[0] = headerByte;
    FullState12Floats4Shorts data = {

            .data1 = InputPos[activeMotorIdx[0]] * 360.0 / motorPos[activeMotorIdx[0]].GetTicksPerRev(),
            .data2 = InputPos[activeMotorIdx[1]] * 360.0 / motorPos[activeMotorIdx[1]].GetTicksPerRev(),
            .data3 = InputPos[activeMotorIdx[2]] * 360.0 / motorPos[activeMotorIdx[2]].GetTicksPerRev(),
            .data4 = InputPos[activeMotorIdx[3]] * 360.0 / motorPos[activeMotorIdx[3]].GetTicksPerRev(),

            .data5 = Input[activeMotorIdx[0]],
            .data6 = Input[activeMotorIdx[1]],
            .data7 = Input[activeMotorIdx[2]],
            .data8 = Input[activeMotorIdx[3]],
            .data9 = (float) 0,
            .data10 = (float) 0,
            .data11 = (float) 0,
            .data12 = (float) 0,

            .current1 = (short) 0,
            .current2 = (short) 0,
            .current3 = (short) 0,
            .current4 = (short) 0,

    };

    sb_set_register(sendBuffer, REG_RETURN_VALS);
    sb_set_type(sendBuffer, TYPE_STREAM);
    sb_set_data_FullState12Floats4Shorts(sendBuffer, data);
    sb_stuff(sendBuffer);
    uint8_t bufferLen = sb_get_length(sendBuffer);
//  DEBUG_SERIAL.println(bufferLen);
//    printSendBuffer(bufferLen);
    Serial.write(sendBuffer, bufferLen + 2);
}



void recvFromSerial() {
    while (Serial.available() > 0) {
//        DEBUG_SERIAL.println("incoming");
        switch (curState) {
            case readHeaderByte:
                rc = Serial.read();
                if (rc == headerByte) {
                    curState = readLengthByte;
                    idx = 0;
                    receiveBuffer[0] = rc;
                }
                break;

            case readLengthByte:
                incomingLength = Serial.read();
                if (incomingLength <= 0) {
                    curState = readError;
                } else {
                    receiveBuffer[1] = incomingLength;
                    curState = readDataByte;
                }
                break;

            case readDataByte:
                rc = Serial.read();
                if (rc == headerByte) {
                    curState = readError;
                } else {
                    receiveBuffer[idx + 2] = rc;
                    idx++;
                    if (idx == incomingLength) {
                        processReceiveBuffer();
                        curState = readHeaderByte;
                    }
                }
                break;

            case readError:
                curState = readHeaderByte;
                break;
        }
    }
}

void processReceiveBuffer() {
    sb_unstuff(receiveBuffer);
    uint8_t recLength = sb_get_length(receiveBuffer);
    regAddr = sb_get_register(receiveBuffer);
    SBTypes recType = sb_get_type(receiveBuffer);
    char *recData = sb_get_data_str(receiveBuffer);
    switch (regAddr) {
        case REG_INIT:
            // Not implemented (yet?)
            break;
        case REG_FIRM_VERSION:
            // Not implemented (yet?)
            break;
        case REG_RETURN_VALS:
            sendToSerial();
            break;
        default:
            if (regAddr >= REG_M1_START && regAddr < REG_RETURN_VALS) {
                uint16_t motorAddr = regAddr - REG_M1_START;
                uint16_t motorID = motorAddr / motorAddrSpace;
                uint16_t relativeMotorAddr = motorAddr - motorID * motorAddrSpace;
                processMotors(motorID, relativeMotorAddr, recData, recLength, recType);
            }
    }
}

void printReceiveBuffer() {
    DEBUG_SERIAL.print("print receive buffer: ");
    for (unsigned char buffer_byte : receiveBuffer) {
        DEBUG_SERIAL.print(buffer_byte);
        DEBUG_SERIAL.print(",");
    }
    DEBUG_SERIAL.println();
}

void printSendBuffer(uint8_t bufLen) {
    for (uint8_t i = 0; i < bufLen; i++) {
        auto buffer_byte = sendBuffer[i];
        Serial.write(buffer_byte);
        DEBUG_SERIAL.print(buffer_byte);
        DEBUG_SERIAL.print(",");
    }
    DEBUG_SERIAL.println();
}

void processMotors(uint16_t motorID, uint16_t relativeMotorAddr, char *data, uint8_t recLength, SBTypes recType) {
    switch (relativeMotorAddr) {
        case REG_REL_EN: {
            char msg_data[1];
            std::copy(data, data + 1, msg_data);
            bool ifEnable = *(bool *) msg_data;
            motor[motorID].Stop(!ifEnable); // Enable pins not connected on motor driver, motor coast
        }
            break;
        case REG_REL_POS: {
            if (motorID == 12) {
                char msg_data[32];
                std::copy(data, data + 32, msg_data);
                for (int iid = 0; iid < 8; iid++) {
                    float rotateDegrees = *(float *) (msg_data + iid * sizeof(float));
                    double rotateTicks = rotateDegrees / 360.0 * motor[iid].GetTicksPerRev();
                    motorPos[iid].SetGoalPos(rotateTicks);
                }
            }
            else {
                char msg_data[8];
                std::copy(data, data + 8, msg_data);
                double rotateDegrees = *(double *) msg_data;
                double rotateTicks = rotateDegrees / 360.0 * motor[motorID].GetTicksPerRev();
                motorPos[motorID].SetGoalPos(rotateTicks);

//            printState(motorID, rotateDegrees, " set pos: ");
            }
        }
            break;

        case REG_REL_SPEED:
            if (motorID == 12) {
                char msg_data[32];
                std::copy(data, data + 32, msg_data);
                for (int iid = 0; iid < 8; iid++) {
                    float msg = *(float *) (msg_data + iid * sizeof(float));
                    if (motorCtrlMode[iid] == vel_ctrl) {
                        motor[iid].SetGoalVelocity(msg);
                    } else {
                        double rotateTicks = msg / 360.0 * motor[iid].GetTicksPerRev();
                        motorPos[iid].SetGoalPos(rotateTicks);
                    }

                }
            }
            else {
                char msg_data[8];
                std::copy(data, data + 8, msg_data);
                double vel = *(double *) msg_data;

                motor[motorID].SetGoalVelocity(vel);
            }
            break;

        case REG_REL_KP: {
                char msg_data[8];
                std::copy(data, data + 8, msg_data);
                motor[motorID].SetKp(*(double *) msg_data);

        }
            break;
        case REG_REL_KI: {
                char msg_data[8];
                std::copy(data, data + 8, msg_data);
                motor[motorID].SetKi(*(double *) msg_data);
        }
            break;
        case REG_REL_KD: {

                char msg_data[8];
                std::copy(data, data + 8, msg_data);
                motor[motorID].SetKd(*(double *) msg_data);

        }
            break;
        case REG_REL_MOVE:
            printError(relativeMotorAddr, motorID);
            break;
        case REG_REL_STOP: // Exactly the same as enable
        {
            char msg_data[1];
            std::copy(data, data + 1, msg_data);
            bool ifEnable = *(bool *) msg_data;

            motor[motorID].Stop(
                    !ifEnable); // Enable pins not connected on motor driver, use this function as stop


        }
            break;
        case REG_REL_GOAL:
            printError(relativeMotorAddr, motorID);
            break;
        case REG_REL_INIT: {
            char msg_data[32];
            std::copy(data, data + 32, msg_data);
            double ticksPerRev = *(double *) (msg_data);
            double Kp = *(double *) (msg_data + sizeof(double));
            double Ki = *(double *) (msg_data + 2 * sizeof(double));
            double Kd = *(double *) (msg_data + 3 * sizeof(double));
            DEBUG_SERIAL.print("motor ");
            DEBUG_SERIAL.print(motorID);
            DEBUG_SERIAL.print(" init : ");
            DEBUG_SERIAL.print(ticksPerRev);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print(Kp);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print(Ki);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.println(Kd);

            motor[motorID].Init(ticksPerRev, Kp, Ki, Kd);
            motorPos[motorID].Init(ticksPerRev, Kp, Ki, Kd);
            activeMotorIdx[nMotorActivated] = motorID;
            nMotorActivated++;
            nMotorActivated %= 4;

        }
            break;

        case REG_REL_MODE: {
            if (motorID == 12) {
                char msg_data[32];
                std::copy(data, data + 32, msg_data);
                for (int iid = 0; iid < 8; iid++) {
                    controlMode thisMode = *(controlMode *) (msg_data + iid * sizeof(controlMode));
                    motorCtrlMode[iid] = thisMode;
                }
            } else {
                char msg_data[4];
                std::copy(data, data + 4, msg_data);
                motorCtrlMode[motorID] = *(controlMode *) msg_data;
            }

        }
            break;
        default:
            printError(relativeMotorAddr, -1);
            break;
    }
}

void sb_unstuff(uint8_t *msg) {
    uint8_t stuffing = 2;
    while (msg[stuffing] != 0xFF) {
        uint8_t tmp = msg[stuffing];
        msg[stuffing] = headerByte;
        stuffing = tmp;
    }
    msg[stuffing] = headerByte;
}

void sb_stuff(uint8_t *msg) {
    int len = sb_get_length(msg) + 2;
    uint8_t stuffing = 2;

    for (uint8_t i = 1; i < len; ++i) {
        if (msg[i] == headerByte) {
            msg[stuffing] = i;
            stuffing = i;
        }
    }
    msg[stuffing] = 0xFF;
}



void printError(uint16_t relativeMotorAddr, uint16_t motorID) {
    DEBUG_SERIAL.print("Error! State: ");
    DEBUG_SERIAL.print(relativeMotorAddr);
    DEBUG_SERIAL.print("motorID: ");
    DEBUG_SERIAL.println(motorID);
}

void printState(uint16_t motorID, uint16_t val, const String &state) {
    DEBUG_SERIAL.print("motor ");
    DEBUG_SERIAL.print(motorID);
    DEBUG_SERIAL.print(state);
    DEBUG_SERIAL.println(val);
}

void blinkNTimes(uint8_t num) {
    for (int i = 0; i < num; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    delay(500);
}