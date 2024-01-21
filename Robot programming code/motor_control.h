// Motor Control Library

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initMotorControl();
void setMotor(unsigned int n, unsigned int dutyCycle, bool dir);
#endif
