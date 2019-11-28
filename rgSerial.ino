#include <Arduino.h>
#include "sb_motor.h"
#include "sb_message.h"
#include "sb_dynamixel.h"
#include <algorithm>
#include <iterator>

#define DEBUG_SERIAL Serial5
#define ANALOG_RES 12
#define BIT2DEG  180.0/pow(2.0, (float)ANALOG_RES)
#define DEG2BIT  pow(2.0, (float)ANALOG_RES)/180.0

/* Communication */
const byte numBytes = 64;
byte recBuffer[numBytes];
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

/* DC Motors*/
SB_Motor motor[6] = {
        SB_Motor(0, 2774.64, 16, 0, 0.3, Input, Output, Setpoint),
        SB_Motor(1, 2774.64, 16, 0, 0.3, Input, Output, Setpoint),
        SB_Motor(2, 2774.64, 16, 0, 0.3, Input, Output, Setpoint),
        SB_Motor(3, 2774.64, 16, 0, 0.3, Input, Output, Setpoint),
        SB_Motor(4, 2774.64, 16, 0, 0.3, Input, Output, Setpoint),
        SB_Motor(5, 2774.64, 16, 0, 0.3, Input, Output, Setpoint)
};

/* Dynamixels */
SB_Dynamixel baseMotor[3] = {
  SB_Dynamixel(100, 0, 0, 0),
  SB_Dynamixel(101, 0, 0, 0),
  SB_Dynamixel(102, 0, 0, 0),
};

/* Joint Vals */
int jointInputs[4][3] = {{0, 0, 0},     // dynamixel current
  {0, 0, 0},     // dynamixel pos
  {0, 0, 0},     // pivot pos
  {0, 0, 0}     // roller pos
}; 

uint32_t startTime = millis();
int myCounter = 0;

void setup() {
    Serial.begin(115200);
    DEBUG_SERIAL.begin(38400);
    analogReadResolution(ANALOG_RES);
    delay(2000);
    pinMode(LED_BUILTIN, OUTPUT);
    blinkNTimes(5);
}

void loop() {
    recvFromSerial();
    for (auto &i : motor) {
        if (i.GetInit() && !i.GetStop()) {
            i.Update();
        }
    }
    updateDynamixels(false);
    sendToSerial();
//    delay(1);
  if (millis() - startTime > 1000) {
    startTime = millis();
    DEBUG_SERIAL.println(myCounter);
    myCounter = 0;
  }
//  else myCounter++;
}

void sendToSerial() {
    sendBuffer[0] = 0x7E;
    Timestamped9Floats data = {
            .time = millis(),
            .data1 = ((float)baseMotor[0].read_position()) * BIT2DEG,
            .data2 = ((float)baseMotor[1].read_position()) * BIT2DEG,
            .data3 = ((float)baseMotor[2].read_position()) * BIT2DEG,
            .data4 = Input[0],
            .data5 = Input[1],
            .data6 = Input[2],
            .data7 = Input[3],
            .data8 = Input[4],
            .data9 = Input[5],
    };
    sb_set_register(sendBuffer, REG_RETURN_POS);
    sb_set_type(sendBuffer, TYPE_STREAM);
    sb_set_data_Timestamped9Floats(sendBuffer, data);
    sb_stuff(sendBuffer);
    uint8_t bufferLen = sb_get_length(sendBuffer);
//    if (myCounter == 100) {
//        DEBUG_SERIAL.print(Input[0]);
//        DEBUG_SERIAL.print(", ");
//        DEBUG_SERIAL.print(Input[1]);
//        DEBUG_SERIAL.print(", ");
//        DEBUG_SERIAL.print(Input[2]);
//        DEBUG_SERIAL.print(", ");
//        DEBUG_SERIAL.println(Input[3]);
//        myCounter = 0;
//    }
    myCounter++;
    Serial.write(sendBuffer, bufferLen + 2);
}

void recvFromSerial() {
    while (Serial.available() > 0) {
        switch (curState) {
            case readHeaderByte:
                rc = Serial.read();
                if (rc == 0x7E) {
                    curState = readLengthByte;
                    idx = 0;
                    recBuffer[0] = rc;
                }
                break;

            case readLengthByte:
                incomingLength = Serial.read();
                if (incomingLength <= 0) {
                    curState = readError;
                } else {
                    recBuffer[1] = incomingLength;
                    curState = readDataByte;
                }
                break;

            case readDataByte:
                rc = Serial.read();
                if (rc == 0x7E) {
                    curState = readError;
                }
                else {
                    recBuffer[idx + 2] = rc;
                    idx++;
                    if (idx == incomingLength) {
                        processRecBuffer();
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

void processRecBuffer() {
    sb_unstuff(recBuffer);
    uint8_t recLength = sb_get_length(recBuffer);
    regAddr = sb_get_register(recBuffer);
    SBTypes recType = sb_get_type(recBuffer);
    char* recData = sb_get_data_str(recBuffer);
    switch (regAddr) {
        case REG_INIT:
              // Not implemented (yet?)
            break;
        case REG_FIRM_VERSION:
              // Not implemented (yet?)
            break;
        default:
            if (regAddr >= REG_M1_EN && regAddr < REG_MOVE) {
                uint16_t motorAddr = regAddr - REG_M1_EN;
                uint16_t motorID = motorAddr / motorAddrSpace;
                uint16_t relativeMotorAddr = motorAddr - motorID * motorAddrSpace;
                if (motorID < 3) processDynamixels(motorID, relativeMotorAddr, recData, recLength, recType);
                else processMotors(motorID-3, relativeMotorAddr, recData, recLength, recType);
            }
    }
}

void printRecBuffer() {
    for (uint8_t i = 0; i < 64; i++) {
        auto buffer_byte = recBuffer[i];
        Serial.write(buffer_byte);
    }
}

void printSendBuffer(uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        auto buffer_byte = sendBuffer[i];
        Serial.write(buffer_byte);
        DEBUG_SERIAL.print(buffer_byte);
        DEBUG_SERIAL.print(",");
    }
    DEBUG_SERIAL.println();
}

void processMotors(uint16_t motorID, uint16_t relativeMotorAddr, char* data, uint8_t recLength, SBTypes recType) {
    switch (relativeMotorAddr) {
        case REG_REL_EN:
        {
            char msg_data[1];
            std::copy(data, data + 1, msg_data);
            bool ifEnable = *(bool*) msg_data; 
            motor[motorID].Stop(!ifEnable); // Enable pins not connected on motor driver, use this function as stop
        }
            break;
        case REG_REL_POS:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            double rotateDegrees = *(double*)msg_data / 360.0 * motor[motorID].GetTicksPerRev();
            motor[motorID].SetGoalPos(rotateDegrees);
            DEBUG_SERIAL.print("motor ");
            DEBUG_SERIAL.print(motorID);
            DEBUG_SERIAL.print(" setPos : ");
            DEBUG_SERIAL.println(rotateDegrees);
        }
            break;
        case REG_REL_CURRENT:
            break;
        case REG_REL_SPEED:
            break;
        case REG_REL_KP:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            motor[motorID].SetKp(*(double*)msg_data);
        }
            break;
        case REG_REL_KI:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            motor[motorID].SetKi(*(double*)msg_data);
        }
            break;
        case REG_REL_KD:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            motor[motorID].SetKd(*(double*)msg_data);
        }
            break;
        case REG_REL_MOVE:
            break;
        case REG_REL_STOP:
        {
            char msg_data[1];
            std::copy(data, data + 1, msg_data);
            bool ifStop = *(bool*) msg_data;
            motor[motorID].Stop(ifStop);
            DEBUG_SERIAL.print("motor ");
            DEBUG_SERIAL.print(motorID);
            DEBUG_SERIAL.print(" stop : ");
            DEBUG_SERIAL.println(ifStop);
        }
            break;
        case REG_REL_GOAL:
            break;
        case REG_REL_INIT:
        {
            char msg_data[32];
            std::copy(data, data + 32, msg_data);
            double ticksPerRev = *(double*)(msg_data);
            double Kp = *(double*) (msg_data + sizeof(double));
            double Ki = *(double*) (msg_data + 2 * sizeof(double));
            double Kd = *(double*) (msg_data + 3 * sizeof(double));
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
        }
            break;
        default:
          DEBUG_SERIAL.println("motor default state.");
          break;
    }
}

void processDynamixels(uint16_t motorID, uint16_t relativeMotorAddr, char* data, uint8_t recLength, SBTypes recType) {
    switch (relativeMotorAddr) {
        case REG_REL_EN:
        {
            char msg_data[1];
            std::copy(data, data + 1, msg_data);
            bool ifEnable = *(bool*) msg_data;
            baseMotor[motorID].torque_en(ifEnable); // Motor torque on/off
        }
            break;
        case REG_REL_POS:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            double rotateDegrees = *(double*)msg_data / 360.0 * motor[motorID].GetTicksPerRev();
            jointInputs[1][motorID] = (int)(rotateDegrees * DEG2BIT);
//            baseMotor[motorID].set_goal_position((int)(rotateDegrees * DEG2BIT));
//            baseMotor[motorID].position_goal();
            DEBUG_SERIAL.print("dynamixel ");
            DEBUG_SERIAL.print(motorID);
            DEBUG_SERIAL.print(" setPos : ");
            DEBUG_SERIAL.println(rotateDegrees);
        }
            break;
        case REG_REL_CURRENT:
        {
            char msg_data[8];
            std::copy(data, data + 8, msg_data);
            int currentBit = *(int*)msg_data;
            jointInputs[0][motorID] = currentBit;
//            baseMotor[motorID].set_goal_current(currentBit);
//            baseMotor[motorID].current_goal();
            DEBUG_SERIAL.print("dynamixel ");
            DEBUG_SERIAL.print(motorID);
            DEBUG_SERIAL.print(" set current : ");
            DEBUG_SERIAL.println(currentBit);
        }
            break;
        case REG_REL_SPEED:
            break;
        case REG_REL_KP:
            break;
        case REG_REL_KI:
            break;
        case REG_REL_KD:
            break;
        case REG_REL_MOVE:
            break;
        case REG_REL_STOP:
            break;
        case REG_REL_GOAL:
            break;
        case REG_REL_INIT:
        {
            char msg_data[32];
            std::copy(data, data + 32, msg_data);
            double ticksPerRev = *(double*)(msg_data);
            double Kp = *(double*) (msg_data + sizeof(double));
            double Ki = *(double*) (msg_data + 2 * sizeof(double));
            double Kd = *(double*) (msg_data + 3 * sizeof(double));
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
            baseMotor[motorID].init();
        }
            break;
        default:
          DEBUG_SERIAL.println("dynamixel default state.");
          break;
    }
}

void sb_unstuff(uint8_t* msg) {
    uint8_t stuffing = 2;
    while (msg[stuffing] != 0xFF) {
        uint8_t tmp = msg[stuffing];
        msg[stuffing] = 0x7E;
        stuffing = tmp;
    }
    msg[stuffing] = 0x7E;
}

void sb_stuff(uint8_t* msg) {
    int len = sb_get_length(msg) + 2;
    uint8_t stuffing = 2;

    for (uint8_t i = 1; i < len; ++i) {
        if (msg[i] == 0x7E) {
            msg[stuffing] = i;
            stuffing = i;
        }
    }
    msg[stuffing] = 0xFF;
}

void updateDynamixels(bool ifPrint) {
  for (int i = 0; i < 3; i++) {
    baseMotor[i].set_goal_current(map(jointInputs[0][i], 1, 4095, 0, 1193));
    baseMotor[i].current_goal();
    baseMotor[i].set_goal_position(map(jointInputs[1][i], 1, 4095, 1536, 2560));
    baseMotor[i].position_goal();
    if (ifPrint) {
      DEBUG_SERIAL.print(baseMotor[i].get_goal_current());
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(baseMotor[i].get_goal_position());
      DEBUG_SERIAL.print(", ");
    }
  }
  if (ifPrint) DEBUG_SERIAL.println();
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
