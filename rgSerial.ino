#include <Arduino.h>
#include "sb_motor.h"
#include "sb_message.h"
#include "sb_dynamixel.h"
#include <algorithm>
#include <iterator>

#define DEBUG_SERIAL Serial5
#define ANALOG_RES 12
#define BIT2DEG  360.0/pow(2.0, (float)ANALOG_RES)
#define DEG2BIT  pow(2.0, (float)ANALOG_RES)/360.0

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
  {0, 0, 0},     // pivot pos (not used in this code)
  {0, 0, 0}     // roller pos (not used in this code)
};

const int posLowLimDeg = 135;
const int posHighLimDeg = posLowLimDeg + 90;
const int posLowLimBits = posLowLimDeg * DEG2BIT;
const int posHighLimBits = posHighLimDeg * DEG2BIT;
const int currentLowLimBits = 0;
const int currentHighLimBits = 1193;

uint32_t startTime = millis();
int myCounter = 0;

void setup() {
//  Serial.begin(115200);
  Serial.begin(1000000);
  DEBUG_SERIAL.begin(115200);
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
  //  sendToSerial();
  //    delay(1);
  //  if (millis() - startTime > 1000) {
  //    startTime = millis();
  //    DEBUG_SERIAL.println(myCounter);
  //    myCounter = 0;
  //  }
  //  else myCounter++;
}

void sendToSerial() {
  sendBuffer[0] = 0x7E;
  Timestamped12Floats data = {
    .time = millis(),
    .data1 = ((float)baseMotor[0].read_position()) * BIT2DEG,
    .data2 = ((float)baseMotor[1].read_position()) * BIT2DEG,
    .data3 = ((float)baseMotor[2].read_position()) * BIT2DEG,
    .data4 = Input[0] * 360.0 / motor[0].GetTicksPerRev(),
    .data5 = Input[1] * 360.0 / motor[1].GetTicksPerRev(),
    .data6 = Input[2] * 360.0 / motor[2].GetTicksPerRev(),
    .data7 = Input[3] * 360.0 / motor[3].GetTicksPerRev(),
    .data8 = Input[4] * 360.0 / motor[4].GetTicksPerRev(),
    .data9 = Input[5] * 360.0 / motor[5].GetTicksPerRev(),
    .data10 = ((float)baseMotor[0].read_current()),
    .data11 = ((float)baseMotor[1].read_current()),
    .data12 = ((float)baseMotor[2].read_current())
  };
  sb_set_register(sendBuffer, REG_RETURN_VALS);
  sb_set_type(sendBuffer, TYPE_STREAM);
  sb_set_data_Timestamped12Floats(sendBuffer, data);
  sb_stuff(sendBuffer);
  uint8_t bufferLen = sb_get_length(sendBuffer);
//  DEBUG_SERIAL.println(bufferLen);
//  printSendBuffer(bufferLen);
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
    case REG_RETURN_VALS:
      sendToSerial();
      break;
    default:
      if (regAddr >= REG_M1_EN && regAddr < REG_MOVE) {
        uint16_t motorAddr = regAddr - REG_M1_EN;
        uint16_t motorID = motorAddr / motorAddrSpace;
        uint16_t relativeMotorAddr = motorAddr - motorID * motorAddrSpace;
        processMotors(motorID, relativeMotorAddr, recData, recLength, recType);
      }
  }
}

void printRecBuffer() {
  for (uint8_t i = 0; i < 64; i++) {
    auto buffer_byte = recBuffer[i];
    Serial.write(buffer_byte);
  }
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

void processMotors(uint16_t motorID, uint16_t relativeMotorAddr, char* data, uint8_t recLength, SBTypes recType) {
  switch (relativeMotorAddr) {
    case REG_REL_EN:
      {
        char msg_data[1];
        std::copy(data, data + 1, msg_data);
        bool ifEnable = *(bool*) msg_data;
        if (motorID < 3) baseMotor[motorID].torque_en(ifEnable); // Motor torque on/off
        else motor[motorID - 3].Stop(!ifEnable); // Enable pins not connected on motor driver, use this function as stop
      }
      break;
    case REG_REL_POS:
      {
        char msg_data[8];
        std::copy(data, data + 8, msg_data);
        double rotateDegrees = *(double*)msg_data;
        
        if (motorID < 3) jointInputs[1][motorID] = (int)(rotateDegrees * DEG2BIT);
        else {
          double rotateTicks = rotateDegrees / 360.0 * motor[motorID - 3].GetTicksPerRev();
          motor[motorID - 3].SetGoalPos(rotateTicks);
        }
//        printState(motorID, rotateDegrees, " set pos: ");
      }
      break;
    case REG_REL_CURRENT:
      if (motorID < 3) {
        char msg_data[8];
        std::copy(data, data + 8, msg_data);
        int currentBit = *(int*)msg_data;
        jointInputs[0][motorID] = currentBit;
//        printState(motorID, currentBit, " set current: ");
      } else {
        printError(relativeMotorAddr,  motorID);
      }
      break;
    case REG_REL_SPEED:
      printError(relativeMotorAddr,  motorID);
      break;
    case REG_REL_KP:
      {
        if (motorID < 3) printError(relativeMotorAddr,  motorID);
        else {
          char msg_data[8];
          std::copy(data, data + 8, msg_data);
          motor[motorID - 3].SetKp(*(double*)msg_data);
        }

      }
      break;
    case REG_REL_KI:
      {
        if (motorID < 3) printError(relativeMotorAddr,  motorID);
        else {
          char msg_data[8];
          std::copy(data, data + 8, msg_data);
          motor[motorID - 3].SetKi(*(double*)msg_data);
        }
      }
      break;
    case REG_REL_KD:
      {
        if (motorID < 3) printError(relativeMotorAddr,  motorID);
        else {
          char msg_data[8];
          std::copy(data, data + 8, msg_data);
          motor[motorID - 3].SetKd(*(double*)msg_data);
        }
      }
      break;
    case REG_REL_MOVE:
      printError(relativeMotorAddr,  motorID);
      break;
    case REG_REL_STOP: // Exactly the same as enable
      {
        char msg_data[1];
        std::copy(data, data + 1, msg_data);
        bool ifEnable = *(bool*) msg_data;
        if (motorID < 3) baseMotor[motorID].torque_en(ifEnable); // Motor torque on/off
        else motor[motorID - 3].Stop(!ifEnable); // Enable pins not connected on motor driver, use this function as stop
      }
      break;
    case REG_REL_GOAL:
      printError(relativeMotorAddr,  motorID);
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
        if (motorID < 3) baseMotor[motorID].init();
        else motor[motorID-3].Init(ticksPerRev, Kp, Ki, Kd);
      }
      break;
    default:
      printError(relativeMotorAddr,  99);
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
    jointInputs[1][i] = min(jointInputs[1][i], posHighLimBits); // set pos
    jointInputs[1][i] = max(jointInputs[1][i], posLowLimBits);  // go to pos
    //    DEBUG_SERIAL.println(jointInputs[1][i]);
    baseMotor[i].set_goal_position(jointInputs[1][i]);
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

void printError(uint16_t relativeMotorAddr, uint16_t motorID) {
  DEBUG_SERIAL.print("Error! State: ");
  DEBUG_SERIAL.print(relativeMotorAddr);
  DEBUG_SERIAL.print("motorID: ");
  DEBUG_SERIAL.println(motorID);
}

void printState(uint16_t motorID, uint16_t val, const String& state) {
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
