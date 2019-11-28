
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sb_motor.h"


SB_Motor::SB_Motor(int motorID, float ticksPerRev, float Kp, float Ki, float Kd,
	float* Input, float* Output, float* Setpoint): 
	myPID(Input + motorID, Output + motorID, Setpoint + motorID, Kp, Ki, Kd, DIRECT){
	
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
	m_enablePin = enablePin[m_motorID];

}

void SB_Motor::Init(float ticksPerRev, float Kp, float Ki, float Kd) {
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
  	digitalWrite(m_enablePin, HIGH);
  	m_init = true;

}

void SB_Motor::Update() {
	myPID.Compute();
	// Serial.println(*m_Output);
	pwmValN = static_cast<int>((abs(*m_Output) - *m_Output) / 2);
  	pwmValP = static_cast<int>((abs(*m_Output) + *m_Output) / 2);
  	
  	analogWrite(m_pwmPinN, pwmValN);
  	analogWrite(m_pwmPinP, pwmValP);
}

void SB_Motor::Stop(bool ifStop) {
	m_stop = ifStop;
	if (m_stop) {
		analogWrite(m_pwmPinN, 0);
		analogWrite(m_pwmPinP, 0);
	}
}

void SB_Motor::Enable(bool ifEnable) {
	if (ifEnable) digitalWrite(m_enablePin, HIGH);
	else digitalWrite(m_enablePin, LOW);
}

void SB_Motor::SetTicksPerRev(float ticksPerRev) {m_ticksPerRev = ticksPerRev;}

void SB_Motor::SetKp(float Kp) {
	m_Kp = Kp;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor::SetKi(float Ki) {
	m_Ki = Ki;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor::SetKd(float Kd) {
	m_Kd = Kd;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor::SetGoalPos(float Setpoint) {*m_Setpoint = Setpoint;}

void SB_Motor::Off() {
	pwmValN = 0;
  	pwmValP = 0;
	analogWrite(m_pwmPinN, pwmValN);
  	analogWrite(m_pwmPinP, pwmValP);
}

float SB_Motor::GetKp() {return m_Kp;}
float SB_Motor::GetKi() {return m_Ki;}
float SB_Motor::GetKd() {return m_Kd;}
float SB_Motor::GetGoalPos() {return *m_Setpoint;}
float SB_Motor::GetTicksPerRev() {return m_ticksPerRev;}
bool SB_Motor::GetInit() {return m_init;}
bool SB_Motor::GetStop() {return m_stop;}

/*
static void SB_Motor::updateEncoder(){
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


