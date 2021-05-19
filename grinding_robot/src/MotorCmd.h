#pragma once
#include <QObject>

typedef unsigned char byte;

enum MOTOR_TYPE   { DRIVE_MOTOR=1, STEER_MOTOR=2, M1010=3, M4215=4, AZD_KD=5};
enum STATE_CMD    { WRITE_CMD=1, READ_CMD = 2};
enum CONTROL_CMD  { SPEED=1, ACCELERATION=2, ENCODER=3, STATUS=4, IDMOTOR=5,STOPMOTOR=6,RUNMOTOR=7,ANGLEOFFSET=8 /*max speed + angle*/, INCREMENTALPOSITION=9, ENABLEMOSBUS=10, HOME=11
};
void CANInitializationFrame(unsigned char *initializitionFrame);

struct MotorData {
	int idMotor;
	double value;
	MOTOR_TYPE motor_type;
	STATE_CMD state_cmd;
	CONTROL_CMD command_type;
	double maxSpeed = 0;
	unsigned char spinDirection = 0; // 0: CCW, 1: CW Nutri motor
	unsigned char timeToReachValue = 5; // for nutri motor RS485

};

class MotorCmd
{
public:
	MotorCmd();
	~MotorCmd();

	bool getFrameData(std::vector<byte> &dataFrame, int &len, MotorData& motorData);

	bool encodeFrameData(std::vector<byte> &dataFrame, MotorData& motorData);
};
