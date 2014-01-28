//Robot.cpp

//Includes
#include <cmath>
#include "WPILib.h"
#include "Values.h"
#include "Meccanum.h"

//Main Class
class Robot : public SimpleRobot
{
private:
	Joystick driveJoy;
	Joystick shooterJoy;
	
	Victor frMotor;
	Victor flMotor;
	Victor brMotor;
	Victor blMotor;
	Encoder frEnc;
	Encoder flEnc;
	Encoder brEnc;
	Encoder blEnc;
	PIDVelocityMotor fr;
	PIDVelocityMotor fl;
	PIDVelocityMotor br;
	PIDVelocityMotor bl;
	
	Meccanum drive;
	double forward;
	double right;
	double clockwise;
	double scalar;

public:
	Robot(void):
		driveJoy(DRIVE_JOYSTICK),
		shooterJoy(SHOOTER_JOYSTICK),
		frMotor(DRIVE_VICTOR_SIDECARS[0], FR_DRIVE_VICTOR),
		flMotor(DRIVE_VICTOR_SIDECARS[1], FL_DRIVE_VICTOR),
		brMotor(DRIVE_VICTOR_SIDECARS[2], BR_DRIVE_VICTOR),
		blMotor(DRIVE_VICTOR_SIDECARS[3], BL_DRIVE_VICTOR),
		frEnc(FR_DRIVE_ENCODER_SIDECAR, FR_DRIVE_ENCODER_A, FR_DRIVE_ENCODER_SIDECAR, FR_DRIVE_ENCODER_B, FR_DRIVE_ENCODER_B),
		flEnc(FL_DRIVE_ENCODER_SIDECAR, FL_DRIVE_ENCODER_A, FL_DRIVE_ENCODER_SIDECAR, FL_DRIVE_ENCODER_B, FL_DRIVE_ENCODER_B),
		brEnc(BR_DRIVE_ENCODER_SIDECAR, BR_DRIVE_ENCODER_A, BR_DRIVE_ENCODER_SIDECAR, BR_DRIVE_ENCODER_B, BR_DRIVE_ENCODER_B),
		blEnc(BL_DRIVE_ENCODER_SIDECAR, BL_DRIVE_ENCODER_A, BL_DRIVE_ENCODER_SIDECAR, BL_DRIVE_ENCODER_B, BL_DRIVE_ENCODER_B),
		fr("FR", frMotor, frEnc, FR_DRIVE_PID),
		fl("FL", flMotor, flEnc, FL_DRIVE_PID),
		br("BR", brMotor, brEnc, BR_DRIVE_PID),
		bl("BL", blMotor, blEnc, BL_DRIVE_PID),
		drive(fr, fl, br, bl),
		forward(0.0),
		right(0.0),
		clockwise(0.0),
		scalar(1.0)
	{
		frEnc.SetDistancePerPulse(FR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		flEnc.SetDistancePerPulse(FL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		brEnc.SetDistancePerPulse(BR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		blEnc.SetDistancePerPulse(BL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		frEnc.SetReverseDirection(FR_DRIVE_ENCODER_REVERSE);
		flEnc.SetReverseDirection(FL_DRIVE_ENCODER_REVERSE);
		brEnc.SetReverseDirection(BR_DRIVE_ENCODER_REVERSE);
		blEnc.SetReverseDirection(BL_DRIVE_ENCODER_REVERSE);
		frEnc.Start();
		flEnc.Start();
		brEnc.Start();
		blEnc.Start();
	}
	
	/**
	 * Kill the watchdog so it doesn't interfere with the program.
	 */
	void RobotInit(void) {
		Watchdog().SetEnabled(false);
	}
	
	/**
	 * Print a message stating that the robot is disabled.
	 */
	void Disabled(void) {
		printf("Robot is disabled!\n");
	}

	/**
	 * Drive left & right motors for 2 seconds then stop.
	 */
	void Autonomous(void)
	{
		while(IsEnabled() && IsAutonomous()) {
			
		}
	}

	/**
	 * Runs the motors with PID Meccanum drive. 
	 */
	void OperatorControl(void) {
		while(IsEnabled() && IsOperatorControl()) {
			if(fabs(driveJoy.GetRawAxis(2)) < 0.2) {
				forward = 0.0;
			} else {
				forward = driveJoy.GetRawAxis(2);
				forward *= fabs(forward);
				forward *= scalar;
			}
			if(fabs(driveJoy.GetRawAxis(1)) < 0.2) {
				right = 0.0;
			} else {
				right = driveJoy.GetRawAxis(1);
				right *= fabs(right);
				right *= scalar;
			}
			if(fabs(driveJoy.GetRawAxis(3)) < 0.2) {
				clockwise = 0.0;
			} else {
				clockwise = driveJoy.GetRawAxis(3);
				if(clockwise <= 0.5) {
					clockwise *= 0.5;
				} else {
					clockwise *= fabs(clockwise);
				}
				clockwise *= scalar;
			}
			drive.update(forward, right, clockwise);
						//Forward  Right  Clockwise
			//printf("Forward: %f,\tRight: %f,\tClockwise: %f\n", forward, right, clockwise);
			DriverStationLCD *dslcd = DriverStationLCD::GetInstance();
			dslcd->PrintfLine(DriverStationLCD::kUser_Line1, "Forward: %f", forward);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line2, "Right: %f", right);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line3, "Clockwise: %f", clockwise);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line4, "Time: %f", GetTime());
			dslcd->PrintfLine(DriverStationLCD::kUser_Line5, "DriveJoy: X=%f, Y=%f, Z=%f", driveJoy.GetRawAxis(1));
			dslcd->PrintfLine(DriverStationLCD::kUser_Line6, "Y=%f, Z=%f", driveJoy.GetRawAxis(2), driveJoy.GetRawAxis(3));
			dslcd->UpdateLCD();
			
			/*if(driveJoy.getRawButton(1)) {
				drive.update(1.0, 0.0, 0.0);
			} else if(driveJoy.getRawButton(3)) {
				drive.update(0.75, 0.0, 0.0);
			} else if(driveJoy.getRawButton(2)) {
				drive.update(0.5, 0.0, 0.0);
			} else {
				drive.update(0.0, 0.0, 0.0);
			}*/
			
			Wait(0.005);
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {
		while(IsTest()) {
			LiveWindow::GetInstance()->Run();
		}
	}
};

START_ROBOT_CLASS(Robot);
