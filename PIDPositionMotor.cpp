//PIDPositionMotor.cpp

//Includes
#include <cmath>
#include "PIDPositionMotor.h"

//Constructor
PIDPositionMotor::PIDPositionMotor(char * _name,
                                   Victor &_vic,
                                   Encoder &_encoder,
                                   const double _posPID[],
                                   const double _velPID[]) : name(_name),
                                   	   	   	   	   motor(_name, _vic, _encoder, _velPID),
                                                   encoder(_encoder),
                                                   position(0.0),
                                                   target(0.0),
                                                   error(0.0),
                                                   deltaTime(0.0),
                                                   kp(_posPID[0]),
                                                   ki(_posPID[1]),
                                                   kd(_posPID[2]),
                                                   maxVel(_velPID[3]),
                                                   integral(0.0),
												   derivative(0.0),
												   out(0.0),
												   lastError(0.0),
												   lastTime(0)
{
	printf("PIDPositionMotor %s created!\n", name);
}

//Destructor
PIDPositionMotor::~PIDPositionMotor() {}

//Functions

//PIDPositionMotor control
void PIDPositionMotor::run() {
    if(GetTime() - 0.001 > lastTime) {
        position = encoder.GetDistance();
        while(position > 360 || position < -360) {
            position /= 360;
        }
        error = target - encoder.GetDistance();
        if(target == encoder.GetDistance()) {
            integral = 0.0;
        }
        deltaTime = GetTime() - lastTime;

        integral += error * deltaTime;
        derivative = (error - lastError) / deltaTime;
        out = (kp * error) + (ki * integral) + (kd * derivative);
		out *= maxVel;
		if(out >= maxVel) {
			motor.run(maxVel);
		} else if(out <= -maxVel) {
			motor.run(-maxVel);
		} else {
			motor.run(out);
		}
        printf("Name: %s KP: %f Target: %f CurrentPos: %f Error: %f Get(): %ld Out: %f\n", name, kp, target, encoder.GetDistance(), error, encoder.Get(), out);

        lastError = error;
        lastTime = GetTime();
    } else {
    	printf("PIDPositionMotor %s waiting... Time: %f\n", name, GetTime());
    }
}

//PIDPositionMotor control
void PIDPositionMotor::run(double pos) {
    target = pos;
    run();
}

char * PIDPositionMotor::getName() {
	return name;
}
