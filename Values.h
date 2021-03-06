//Values.h

#ifndef VALUES_H
#define VALUES_H

	const int SIDECAR_R = 1;
	const int SIDECAR_L = 2;

	const int DRIVE_JOYSTICK = 1;

	const int FR_DRIVE_VICTOR		= 9;
	const int FL_DRIVE_VICTOR		= 9;
	const int BR_DRIVE_VICTOR		= 10;
	const int BL_DRIVE_VICTOR		= 10;
	const int DRIVE_VICTOR_SIDECARS[4]	= {SIDECAR_R, SIDECAR_L, SIDECAR_R, SIDECAR_L};
										//FR			FL			BR			BL
	const int DRIVE_VICTORS[4]			= {FR_DRIVE_VICTOR, FL_DRIVE_VICTOR, BR_DRIVE_VICTOR, BL_DRIVE_VICTOR};

	const int FR_DRIVE_ENCODER_A	= 3;
	const int FR_DRIVE_ENCODER_B	= 4;
	const int FR_DRIVE_ENCODER_SIDECAR = SIDECAR_R;
	const int FR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1;
	const bool FR_DRIVE_ENCODER_REVERSE = false;

	const int FL_DRIVE_ENCODER_A	= 3;
	const int FL_DRIVE_ENCODER_B	= 4;
	const int FL_DRIVE_ENCODER_SIDECAR = SIDECAR_L;
	const int FL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1;
	const bool FL_DRIVE_ENCODER_REVERSE = false;

	const int BR_DRIVE_ENCODER_A	= 1;
	const int BR_DRIVE_ENCODER_B	= 2;
	const int BR_DRIVE_ENCODER_SIDECAR = SIDECAR_R;
	const int BR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1;
	const bool BR_DRIVE_ENCODER_REVERSE = false;

	const int BL_DRIVE_ENCODER_A	= 1;
	const int BL_DRIVE_ENCODER_B	= 2;
	const int BL_DRIVE_ENCODER_SIDECAR = SIDECAR_L;
	const int BL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1;
	const bool BL_DRIVE_ENCODER_REVERSE = false;

	const int DRIVE_ENCODERS[4][2] = {
										{FR_DRIVE_ENCODER_A, FR_DRIVE_ENCODER_B},
										{FL_DRIVE_ENCODER_A, FL_DRIVE_ENCODER_B},
										{BR_DRIVE_ENCODER_A, BR_DRIVE_ENCODER_B},
										{BL_DRIVE_ENCODER_A, BL_DRIVE_ENCODER_B}
									 };
	const int DRIVE_ENCODER_SIDECARS[4] = {FR_DRIVE_ENCODER_SIDECAR, FL_DRIVE_ENCODER_SIDECAR, BR_DRIVE_ENCODER_SIDECAR, BL_DRIVE_ENCODER_SIDECAR};
	const int DRIVE_ENCODER_DISTANCES_PER_PULSE[4] = {FR_DRIVE_ENCODER_DISTANCE_PER_PULSE, FL_DRIVE_ENCODER_DISTANCE_PER_PULSE, BR_DRIVE_ENCODER_DISTANCE_PER_PULSE, BL_DRIVE_ENCODER_DISTANCE_PER_PULSE};
	const bool DRIVE_ENCODER_REVERSES[4] = {FR_DRIVE_ENCODER_REVERSE, FL_DRIVE_ENCODER_REVERSE, BR_DRIVE_ENCODER_REVERSE, BL_DRIVE_ENCODER_REVERSE};

	const double DRIVE_PID_SCALAR = 1120.0;
	const double FR_DRIVE_PID_KP = 0.05;
	const double FL_DRIVE_PID_KP = 0.05;
	const double BR_DRIVE_PID_KP = 0.05;
	const double BL_DRIVE_PID_KP = 0.05;
	const double FR_DRIVE_PID_KI = 1.0;
	const double FL_DRIVE_PID_KI = 1.0;
	const double BR_DRIVE_PID_KI = 1.0;
	const double BL_DRIVE_PID_KI = 1.0;
	const double FR_DRIVE_PID_KD = 0.0005;
	const double FL_DRIVE_PID_KD = 0.0005;
	const double BR_DRIVE_PID_KD = 0.0005;
	const double BL_DRIVE_PID_KD = 0.0005;
	const double FR_DRIVE_PID_KF = 0.5;
	const double FL_DRIVE_PID_KF = 0.5;
	const double BR_DRIVE_PID_KF = 0.5;
	const double BL_DRIVE_PID_KF = 0.5;
	const double FR_DRIVE_PID[5] = {FR_DRIVE_PID_KP, FR_DRIVE_PID_KI, FR_DRIVE_PID_KD, FR_DRIVE_PID_KF, DRIVE_PID_SCALAR};
	const double FL_DRIVE_PID[5] = {FL_DRIVE_PID_KP, FL_DRIVE_PID_KI, FL_DRIVE_PID_KD, FL_DRIVE_PID_KF, DRIVE_PID_SCALAR};
	const double BR_DRIVE_PID[5] = {BR_DRIVE_PID_KP, BR_DRIVE_PID_KI, BR_DRIVE_PID_KD, BR_DRIVE_PID_KF, DRIVE_PID_SCALAR};
	const double BL_DRIVE_PID[5] = {BL_DRIVE_PID_KP, BL_DRIVE_PID_KI, BL_DRIVE_PID_KD, BL_DRIVE_PID_KF, DRIVE_PID_SCALAR};

#endif //VALUES_H
