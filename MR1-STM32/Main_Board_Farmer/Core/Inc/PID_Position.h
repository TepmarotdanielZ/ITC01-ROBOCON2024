#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Derivative low-pass filter time constant */
	float tau;
	/* Output limits */
	float limMin;
	float limMax;
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;
	/* Sample time (in seconds) */
	float T;
	
	/* Lowpass Filter */
	float alpha;
	float derror[3];
	float error_est[3];
	/* Controller "memory" */
	float integrator[3];
	float prevError[3];			/* Required for integrator */
	float differentiator[3];
	float prevMeasurement[3];		/* Required for differentiator */

	/* Controller output */
	float out[3];
	

} PIDPosition;

void  PID_Position_Init(PIDPosition *pid, int nMotor);
float PID_Position(PIDPosition *pid, float setpoint, float measurement,float Kp,float Ki,float Kd, int i);

#endif
