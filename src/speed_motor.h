#ifndef SPEED_MOTOR_H
#define SPEED_MOTOR_H

#include <stdint.h>
#include <PID_v1_float_micros.h>
#include "sb_teensy_pin_m8.h"

class Speed_Motor 
{
	public:
		

		Speed_Motor(int motorID, float ticksPerRev, float Kp, float Ki, float Kd,
			float* Input, float* Output, float* Setpoint);
		void Init(float ticksPerRev, float Kp, float Ki, float Kd);
//		void Enable(bool ifEnable);
		void Stop(bool ifStop);
		void SetTicksPerRev(float ticksPerRev);
		void SetKp(float Kp);
		void SetKi(float Kp);
		void SetKd(float Kp);
		void SetGoalVelocity(float Setpoint);
		void Off();
		float GetKp();
		float GetKi();
		float GetKd();
		float GetGoalPos();
		float GetTicksPerRev();
		bool GetInit();
		bool GetStop();
		void Update();

		// static void updateEncoder();

		float m_lastSetpoint = 0.0;
		float* m_Setpoint;
		float* m_Input;
		float* m_Output;
		// volatile int lastEncoded = 0;
		// volatile long encoderValue = 0;
		// long lastencoderValue = 0;
		volatile long* m_encoderValue;
		volatile long* m_lastEncoderValue;
		int lastMSB = 0;
		int lastLSB = 0;


	private:
		int m_motorID;
		bool m_init = false;
//		bool m_enable = false;
		bool m_stop = false;
		float m_ticksPerRev = 0.0;
		float m_Kp = 0.0;
		float m_Ki = 0.0;
		float m_Kd = 0.0;

		int m_pwmPinN;
		int m_pwmPinP;
		int m_encA;
		int m_encB;
//		int m_enablePin;

		int pwmValN = 0;
		int pwmValP = 0;
		int pwmValN_last = 0;
		int pwmValP_last = 0;

		const unsigned int velocityCalculationTimeInterval = 5;
		unsigned long currVelocityCalculationTime = 0;
		unsigned long lastVelocityCalculationTime = 0;
		double speedActual = 0;

		// kalman filter
		double kalman_K = 0;                // kalman gain 
		const double kalman_R = 0.5 ;      // error in measurement
		double kalman_Eest = 0;              // error estimate 
		double kalman_pre_Eest = 0;          // error estimate(t-1)
		double kalman_est = 0;              // estimate 
		double kalman_pre_est = 0;           // estimate(t-1)
		double kalman_sensor_meg = 0;        // from analog pin 
		double kalman_p1 = 1; 
		const double kalman_Q = 0.01;

		PID myPID;
};

#endif