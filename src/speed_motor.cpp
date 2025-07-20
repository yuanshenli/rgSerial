
#if ARDUINO >= 100

#include "Arduino.h"

#else
#include "WProgram.h"
#endif

#include "speed_motor.h"


Speed_Motor::Speed_Motor(int motorID, float ticksPerRev, float Kp, float Ki, float Kd,
                   float *Input, float *Output, float *Setpoint) :
        myPID(Input + motorID, Output + motorID, Setpoint + motorID, Kp, Ki, Kd, DIRECT) {

    m_motorID = motorID;
    m_ticksPerRev = ticksPerRev;
    m_Input = Input + motorID;
    m_Output = Output + motorID;
    m_Setpoint = Setpoint + motorID;
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_pwmPinN = pwmPinN[m_motorID];
    m_pwmPinP = pwmPinP[m_motorID];
    m_encA = ENC_A[m_motorID];
    m_encB = ENC_B[m_motorID];

    m_encoderValue = encoderValue + m_motorID;
    m_lastEncoderValue = lastEncoderValue + m_motorID;
//    m_enablePin = enablePin[m_motorID];

}

void Speed_Motor::Init(float ticksPerRev, float Kp, float Ki, float Kd) {
    m_ticksPerRev = ticksPerRev;
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
    myPID.SetTunings(m_Kp, m_Ki, m_Kd);

    pinMode(m_pwmPinN, OUTPUT);
    pinMode(m_pwmPinP, OUTPUT);
    pinMode(m_encA, INPUT_PULLUP);
    pinMode(m_encB, INPUT_PULLUP);
//	Serial.println("Pin mode set");


    switch (m_motorID) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder0, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder1, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder2, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder3, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder3, CHANGE);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder4, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder4, CHANGE);
            break;
        case 5:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder5, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder5, CHANGE);
            break;
        case 6:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder6, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder6, CHANGE);
            break;
        case 7:
            attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder7, CHANGE);
            attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder7, CHANGE);
            break;
        default:
            Serial.println("init default");
            break;
    }
    // attachInterrupt(digitalPinToInterrupt(this->encA), updateEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(this->encB), updateEncoder, CHANGE);
//	Serial.println("Interrupt set");

    myPID.SetOutputLimits(-4095, 4095);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);
//  	Serial.println("PID set");
//    digitalWrite(m_enablePin, HIGH);
    m_init = true;

}

// void Speed_Motor::Update() {
//     myPID.Compute();
//     // Serial.println(*m_Output);
//     pwmValN = static_cast<int>((abs(*m_Output) - *m_Output) / 2);
//     pwmValP = static_cast<int>((abs(*m_Output) + *m_Output) / 2);

//     analogWrite(m_pwmPinN, pwmValN);
//     analogWrite(m_pwmPinP, pwmValP);
// }


void Speed_Motor::Update() {
    myPID.Compute();
    // Serial.println(*m_Output);
    currVelocityCalculationTime = millis();
    if ((currVelocityCalculationTime - lastVelocityCalculationTime) > velocityCalculationTimeInterval) {
        lastVelocityCalculationTime = currVelocityCalculationTime;
        double encoderValueChange = (double)(*m_encoderValue - *m_lastEncoderValue);
        double updateFreq = (1000.0 * 60.0) / (double)velocityCalculationTimeInterval;  // 1/min
        double revs = encoderValueChange / m_ticksPerRev;
        speedActual = revs * updateFreq; // rpm


        kalman_sensor_meg=speedActual;
        kalman_pre_Eest=kalman_p1+kalman_Q;
        kalman_K=kalman_Eest/(kalman_Eest+kalman_R);
        kalman_est=kalman_pre_est+kalman_K*(kalman_sensor_meg-kalman_pre_est);
        kalman_Eest=(1-kalman_K)*kalman_pre_Eest;
        
        //update kalman
        kalman_pre_est=kalman_est;
        kalman_p1=kalman_Eest;
        *m_Input=kalman_est;






        // *m_Input = speedActual;
        *m_lastEncoderValue = *m_encoderValue;
        // Serial.print(*m_Setpoint);
        // Serial.print(',');
        // Serial.println(*m_Input);
        // Serial.println(speedActual);
        // Serial.println(speedActual - *m_Setpoint);
        // Serial.println(*m_Output);
    }
    // Serial.println(speedActual);






    pwmValN = static_cast<int>((abs(*m_Output) - *m_Output) / 2);
    pwmValP = static_cast<int>((abs(*m_Output) + *m_Output) / 2);
    // pwmValN = 105;
    // pwmValP = 0;

    analogWrite(m_pwmPinN, pwmValN);
    analogWrite(m_pwmPinP, pwmValP);
}


void Speed_Motor::Stop(bool ifStop) {
    m_stop = ifStop;
    if (m_stop) {
        analogWrite(m_pwmPinN, 0);
        analogWrite(m_pwmPinP, 0);
    }
}

//void Speed_Motor::Enable(bool ifEnable) {
//    if (ifEnable) digitalWrite(m_enablePin, HIGH);
//    else digitalWrite(m_enablePin, LOW);
//}
void Speed_Motor::SetTicksPerRev(float ticksPerRev) { m_ticksPerRev = ticksPerRev; }

void Speed_Motor::SetKp(float Kp) {
    m_Kp = Kp;
    myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}

void Speed_Motor::SetKi(float Ki) {
    m_Ki = Ki;
    myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}

void Speed_Motor::SetKd(float Kd) {
    m_Kd = Kd;
    myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}

void Speed_Motor::SetGoalVelocity(float Setpoint) { *m_Setpoint = Setpoint; }

void Speed_Motor::Off() {
    pwmValN = 0;
    pwmValP = 0;
    analogWrite(m_pwmPinN, pwmValN);
    analogWrite(m_pwmPinP, pwmValP);
}

float Speed_Motor::GetKp() { return m_Kp; }

float Speed_Motor::GetKi() { return m_Ki; }

float Speed_Motor::GetKd() { return m_Kd; }

float Speed_Motor::GetGoalPos() { return *m_Setpoint; }

float Speed_Motor::GetTicksPerRev() { return m_ticksPerRev; }

bool Speed_Motor::GetInit() { return m_init; }

bool Speed_Motor::GetStop() { return m_stop; }

/*
static void Speed_Motor::updateEncoder(){
  int MSB = digitalRead(this->encA); //MSB = most significant bit
  int LSB = digitalRead(this->encB); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (this->lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) this->encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) this->encoderValue --;
//  if(encoderValue < 0) encoderValue = 0;

  this->lastEncoded = encoded; //store this value for next time
}
*/


