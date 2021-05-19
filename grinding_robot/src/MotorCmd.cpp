#include "MotorCmd.h"

#define MAXIMUM_STEER_SPEED 155*6*100 // maximum 155 rpm with torque, dps
#define MAXIMUM_DRIVE_SPEED 25*10
#define MAXIMUM_M1010_SPEED 3200
#define MAXIMUM_M4215_SPEED 1500


MotorCmd::MotorCmd()
{
}

MotorCmd::~MotorCmd()
{
}
unsigned char CaculateCheckSum(char transmitData[100], unsigned char sizeOfCommand) // checksum for Nutri motor
{
	unsigned char checkSum = 0;
	for (int k = 0; k < sizeOfCommand; k++)
	{
		if (k != 0 && k != 1 && k != 4)
			checkSum += transmitData[k];
	}
	return ~checkSum;
}
unsigned short CRC16(unsigned char *data, unsigned char len)
{
	unsigned short CRC = 0xFFFF;
	for (int k = 0; k < len; k++)
	{
		CRC = CRC ^ (data[k] & 0x00FF);
		for (int i = 0; i < 8; i++)
		{
			unsigned short before = CRC;
			CRC >>= 1;
			if ((before - 2 * CRC) == 1)
				CRC = CRC ^ 0xA001;
			CRC = CRC;
		}
	}
	return CRC;
}
void CreateDataCAN(unsigned char *dataHeader, unsigned short ID, unsigned char data[8], unsigned char motorType)
{
	if (motorType == M4215)
		ID = ID + 0x600;
	dataHeader[0] = 0xAA;
	dataHeader[1] = 0xC8;
	dataHeader[2] = ID & 0xFF;
	dataHeader[3] = (ID >> 8) & 0xFF;
	for (int i = 0; i < 8; i++)
		dataHeader[i + 4] = data[i];
	dataHeader[12] = 0x55;
}
/* This Function return Data Frame
Identifier: 0x140 + ID(1~32) (Steer Motor)
Frame Type: standard frame DLC: 8byte
EX: Turn of Motor
dataFrame[0] = 0x80;
dataFrame[1] = 0x00;
dataFrame[2] = 0x00;
dataFrame[3] = 0x00;
dataFrame[4] = 0x00;
dataFrame[5] = 0x00;
dataFrame[6] = 0x00;
dataFrame[7] = 0x00;
*/
bool MotorCmd::getFrameData(std::vector<byte>& dataFrame, int & len, MotorData & motorData)
{
	if (motorData.state_cmd == WRITE_CMD)
	{
		if (motorData.motor_type == STEER_MOTOR)
		{
			unsigned char dataSend[20] = { 0 };
			len = 0;
			switch (motorData.command_type)
			{
				case SPEED: // updated to RS485
				{
					int speed = 0;
					speed = motorData.value * MAXIMUM_STEER_SPEED/100; // convert from rpm --> dps
					dataSend[0] = 0x3E;
					dataSend[1] = 0xA2;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x04; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					dataSend[5] = speed & 0xFF;
					dataSend[6] = (speed >> 8) & 0xFF;
					dataSend[7] = (speed >> 16) & 0xFF;
					dataSend[8] = (speed >> 24) & 0xFF;
					for (int i = 5; i < 9; i++)
						dataSend[9] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
					len = 13;
				}
					break;
				case ACCELERATION:
				{
				
				}
					break;
				case ENCODER: // updated to RS485 case 2: write current encoder position of the motor into ROM as initial position, to reset, set command angle to 0
				{
					dataSend[0] = 0x3E;
					dataSend[1] = 0x19; 
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x00; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i]; // check sum
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
					len = 5;
				}
					break;
				case STATUS: // updated to RS485
				{
					len = 13;
					dataSend[0] = 0x3E;
					dataSend[1] = 0x9C;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x00; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);

				}
					break;
				case IDMOTOR: // updated to RS485
					// ko dung voi steer motor (RMD X8, set ID manual)
					break;
				case STOPMOTOR: // updated to RS485
				{
					len = 5;
					dataSend[0] = 0x3E;
					dataSend[1] = 0x81;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x00; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
				}
					break;
				case RUNMOTOR: // updated to RS485
				{
					len = 5;
					dataSend[0] = 0x3E;
					dataSend[1] = 0x88;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x00; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
				}
					break;
				case INCREMENTALPOSITION: // updated to RS485
				{
					len = 13;
					int angle = motorData.value * 100; // negative or positive
					unsigned int maxSpeed = motorData.maxSpeed * MAXIMUM_STEER_SPEED / 100; // 1dps/LSB, dps->rpm
					
					dataSend[0] = 0x3E;
					dataSend[1] = 0xA8;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x08; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					dataSend[5] = angle & 0xFF;
					dataSend[6] = (angle >> 8) & 0xFF;
					dataSend[7] = (angle >> 16) & 0xFF;
					dataSend[8] = (angle >> 24) & 0xFF;
					dataSend[9] = maxSpeed & 0xFF;
					dataSend[10] = (maxSpeed >> 8) & 0xFF;
					dataSend[11] = (maxSpeed >> 16) & 0xFF;
					dataSend[12] = (maxSpeed >> 24) & 0xFF;
					for (int i = 5; i < 13; i++)
						dataSend[13] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
				}
					break;
				case ANGLEOFFSET: // updated to RS485, set multi-angle
				{
					int maxSpeed = motorData.maxSpeed * MAXIMUM_STEER_SPEED / 100;

					uint64_t position = 0;
					int angle = motorData.value;
					if (angle < -360)
						angle = -360;
					if (angle > 360)
						angle = 360;
					position = angle * 100 * 6;

					dataSend[0] = 0x3E;
					dataSend[1] = 0xA4;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x0C; // data length
					for (int i = 0; i < 4; i++)
						dataSend[4] += dataSend[i];
					dataSend[5] = position & 0xFF;
					dataSend[6] = (position >> 8) & 0xFF;
					dataSend[7] = (position >> 16) & 0xFF;
					dataSend[8] = (position >> 24) & 0xFF;
					dataSend[9] = (position >> 32) & 0xFF;
					dataSend[10] = (position >> 40) & 0xFF;
					dataSend[11] = (position >> 48) & 0xFF;
					dataSend[12] = (position >> 56) & 0xFF;
					dataSend[13] = maxSpeed & 0xFF;
					dataSend[14] = (maxSpeed >> 8) & 0xFF;
					dataSend[15] = (maxSpeed >> 16) & 0xFF;
					dataSend[16] = (maxSpeed >> 24) & 0xFF;
					for (int i = 5; i < 17; i++)
						dataSend[17] += dataSend[i];
					for (int i = 0; i < (dataSend[3] + 6); i++)
						dataFrame.push_back(dataSend[i]);
					len = 13;
				}
				break;
				default:
					break;
			}
		}
		else if (motorData.motor_type == DRIVE_MOTOR) // Nutri motor
		{
			len = 0; 
			dataFrame.clear();
			switch (motorData.command_type)
			{
				case SPEED:
				{
					char dataSend[50];
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 6;
					dataSend[5] = 0x03; // speed control
					unsigned short speed;
					if (motorData.value < 0)
					{
						dataSend[6] = 0; // CCW
						speed = -motorData.value*MAXIMUM_DRIVE_SPEED/100;
					}
					else if (motorData.value >= 0)
					{
						dataSend[6] = 1; // CW
						speed = motorData.value*MAXIMUM_DRIVE_SPEED/100;
					}
					dataSend[7] = (speed >> 8) & 0xFF;
					dataSend[8] = speed & 0xFF;
					dataSend[9] = motorData.timeToReachValue*10; // o.1s/LSB
					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);

					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				case ACCELERATION:
				{
					// already have, no need to set
				}
				break;
				case ENCODER: // Location initialization
				{
					char dataSend[50];
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 2;
					dataSend[5] = 0x0C;
					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);
					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				case STATUS: // read on/off status
				{
					char dataSend[50];
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 2;
					dataSend[5] = 0xA7; // request position feedback
					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);
					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				case IDMOTOR:
					// ko dung voi drive motor
					break;
				case STOPMOTOR:
				{
					char dataSend[50];
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 3;
					dataSend[5] = 0x0A; //10
					dataSend[6] = 1; // 0: on, 1: off
					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);
					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				case RUNMOTOR:
				{
					char dataSend[50];
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 3;
					dataSend[5] = 0x0A; //10
					dataSend[6] = 0; // 0: on, 1: off
					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);
					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				case ANGLEOFFSET:
				{
					char dataSend[50] = { 0 };
					unsigned short angle;// = motorData.value * 100;
					
					dataSend[0] = 0xFF;
					dataSend[1] = 0xFE;
					dataSend[2] = motorData.idMotor;
					dataSend[3] = 0x06; // data size
					dataSend[5] = 0x02;
					if (motorData.value < 0)
					{
						dataSend[6] = 0;
						angle = -motorData.value <= 360 ? -motorData.value * 100 : 36000;
					}
					else if (motorData.value >= 0)
					{
						dataSend[6] = 1;
						angle = motorData.value <= 360 ? motorData.value * 100 : 36000;
					}
					dataSend[7] = angle & 0xFF;
					dataSend[8] = (angle >> 8) & 0xFF;
					dataSend[9] = motorData.timeToReachValue * 10;

					dataSend[4] = CaculateCheckSum(dataSend, dataSend[3] + 4);
					for (int i = 0; i <= dataSend[3] + 4; i++)
						dataFrame.push_back(dataSend[i]); //data size
				}
				break;
				default:
					break;
				}
		}
		else if (motorData.motor_type == M1010)
		{
			dataFrame.clear();
			unsigned char data[20] = { 0 };
			unsigned char dataWithHeader[20] = {};
			switch (motorData.command_type)
			{
				case ENABLEMOSBUS:
				{
					data[0] = 0xFF;
					data[1] = 0x06;
					data[2] = 0x01;
					data[3] = 0x01;
					data[4] = 0x01;
					data[5] = 0x00;
					data[6] = 0x00;
					data[7] = 0x00;
					CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
					len = 6 + 5;
					for (int i = 0; i < 13; i++)
						dataFrame.push_back(dataWithHeader[i]);
				}
				break;
				case ENCODER:
				{
					data[0] = 0xFF;
					data[1] = 0x16;
					data[2] = 0x04;
					data[3] = 0x01;
					data[4] = 0x03;
					data[5] = 0;
					data[6] = 0;
					data[7] = 0;
					CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
					len = 0;// +5;
					for (int i = 0; i < 13; i++)
						dataFrame.push_back(dataWithHeader[i]);
				}
				break;
				case SPEED:
				{
					int targetSpeed = motorData.maxSpeed * MAXIMUM_M1010_SPEED / 100;
					//send target speed
					data[0] = 0xFF;
					data[1] = 0x06;
					data[2] = 0x02;
					data[3] = 0x01;
					data[4] = targetSpeed & 0xFF;
					data[5] = (targetSpeed >> 8) & 0xFF;
					data[6] = 0x00;
					data[7] = 0x00;
					CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
					len = 6 + 5;
					for (int i = 0; i < 13; i++)
						dataFrame.push_back(dataWithHeader[i]);
				}
				break;
				case ANGLEOFFSET:
				{
					int position = motorData.value * 80 * 32768 / 360;
					//if (position > 360) position = 360;
					//if (position < -360) position = -360;

					data[0] = 0xFF;
					data[1] = 0x06;
					data[2] = 0x16;
					data[3] = 0x02;
					data[4] = position & 0xFF;
					data[5] = (position >> 8) & 0xFF;
					data[6] = (position >> 16) & 0xFF;
					data[7] = (position >> 24) & 0xFF;
					CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
					len = 8 + 5;
					for (int i = 0; i < 13; i++)
						dataFrame.push_back(dataWithHeader[i]);
				}
				break;
				case INCREMENTALPOSITION:
				{
					int position = motorData.value * 80 * 32768 / 360;
					data[0] = 0xFF;
					data[1] = 0x06;
					data[2] = 0x0C;
					data[3] = 0x02;
					data[4] = position & 0xFF;
					data[5] = (position >> 8) & 0xFF;
					data[6] = (position >> 16) & 0xFF;
					data[7] = (position >> 24) & 0xFF;
					CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
					len = 8 + 5;
					for (int i = 0; i < 13; i++)
						dataFrame.push_back(dataWithHeader[i]);
				}
				break;
			}
		}
		else if (motorData.motor_type == M4215)
		{
			dataFrame.clear();
			unsigned char data[20] = { 0 };
			unsigned char dataWithHeader[20] = {};
			switch (motorData.command_type)
			{
			case ENABLEMOSBUS: // changed
			{
				data[0] = 0x2B;//
				data[1] = 0x40;//
				data[2] = 0x60;//
				data[3] = 0x00;//
				data[4] = 0x2F;//
				data[5] = 0x00;//
				data[6] = 0x00;//
				data[7] = 0x00;//
				CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
				len = 4 + 5;
				for (int i = 0; i < 13; i++)
					dataFrame.push_back(dataWithHeader[i]);
			}
			break;
			case ENCODER: // done
			{
				data[0] = 0x40;//
				data[1] = 0x64;//
				data[2] = 0x60;//
				data[3] = 0x00;//
				data[4] = 0x00;//
				data[5] = 0x00;//
				data[6] = 0x00;//
				data[7] = 0x00;//
				CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
				len = 8 + 5;
				for (int i = 0; i < 13; i++)
					dataFrame.push_back(dataWithHeader[i]);
			}
			break;
			case SPEED: // done
			{
				int targetSpeed = motorData.maxSpeed * MAXIMUM_M4215_SPEED / 100;
				//send target speed
				data[0] = 0x23;
				data[1] = 0x81;
				data[2] = 0x60;
				data[3] = 0x00;
				data[4] = targetSpeed & 0xFF;
				data[5] = (targetSpeed >> 8) & 0xFF;
				data[6] = 0x00;
				data[7] = 0x00;
				CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
				len = 4 + 5;
				for (int i = 0; i < 13; i++)
					dataFrame.push_back(dataWithHeader[i]);
			}
			break;
			case ANGLEOFFSET:
			{
				int position = motorData.value * 80 * 32768 / 360;
				if (position > 360) position = 360;
				if (position < -360) position = -360;
				data[0] = 0x23;
				data[1] = 0x7A;
				data[2] = 0x60;
				data[3] = 0x00;
				data[4] = position & 0xFF;
				data[5] = (position >> 8) & 0xFF;
				data[6] = (position >> 16) & 0xFF;
				data[7] = (position >> 24) & 0xFF;
				CreateDataCAN(dataWithHeader, motorData.idMotor, data, motorData.motor_type);
				len = 4 + 5;
				for (int i = 0; i < 13; i++)
					dataFrame.push_back(dataWithHeader[i]);
			}

			break;
			case INCREMENTALPOSITION:
			{
				
			}
			break;
			}
		}
		else if (motorData.motor_type == AZD_KD)
		{
			dataFrame.clear();
			unsigned char data[45] = { 0 };
			switch (motorData.command_type)
			{
			case RUNMOTOR: // changed
			{
				data[0] = motorData.idMotor;
				data[1] = 0x06;
				data[2] = 0x00;
				data[3] = 0x7D;
				data[4] = 0;
				data[5] = 0x08;
				unsigned short crc = CRC16(&data[0], 6);
				data[6] = crc & 0xFF;
				data[7] = (crc >> 8) & 0xFF;

				len = 8;
				for (int i = 0; i < 8; i++)
					dataFrame.push_back(data[i]);
			}
			break;
			case STOPMOTOR: // changed
			{
				data[0] = motorData.idMotor;
				data[1] = 0x06;
				data[2] = 0x00;
				data[3] = 0x7D;
				data[4] = 0x00;
				data[5] = 0x00;
				data[6] = 0x19;
				data[7] = 0xD2;

				len = 8;
				for (int i = 0; i < 8; i++)
					dataFrame.push_back(data[i]);
			}
			break;
			case INCREMENTALPOSITION: // changed
			{
				int pulse = motorData.value * 36000 / 360;
				if (motorData.maxSpeed == 0)
					motorData.maxSpeed = 2000;
				int frequency = motorData.maxSpeed;

				data[0] = motorData.idMotor;
				data[1] = 0x10; // function code
				data[2] = 0x00; // register address
				data[3] = 0x58;
				data[4] = 0; // number of register 
				data[5] = 0x10;
				data[6] = 0x20; // bum of writing bytes
				data[7] = 0; // operation data number
				data[8] = 0;
				data[9] = 0;
				data[10] = 0;
				data[11] = 0; // operation type
				data[12] = 0;
				data[13] = 0;
				data[14] = 2; // NOSETTING = 0, ABSOLUTEPOSITIONING = 1, INCREMENTALPOSITIONINGKZD = 2/*based on position command*/, INCREMENTALPOSITIONINGKZD = 3/*based on position feedback*/ };//...more....
				data[15] = (pulse >> 24); // position
				data[16] = (pulse >> 16) & 0xFF;
				data[17] = (pulse >> 8) & 0xFF;
				data[18] = pulse & 0xFF;
				data[19] = (frequency >> 24); // operating speed (frequency)
				data[20] = (frequency >> 16) & 0xFF;
				data[21] = (frequency >> 8) & 0xFF;
				data[22] = frequency & 0xFF;

				data[23] = 00; // start/changing speed rate 1.5kHz/s
				data[24] = 00;
				data[25] = 0x05;
				data[26] = 0xDC;
				data[27] = 00; // stopping speed rate 1.5kHz/s
				data[28] = 00;
				data[29] = 0x05;
				data[30] = 0xDC;
				data[31] = 00; // operating current = 100%
				data[32] = 00;
				data[33] = 0x03;
				data[34] = 0xE8;
				data[35] = 00; // trigger all = 1 all date update
				data[36] = 00;
				data[37] = 00;
				data[38] = 0x1;
				unsigned short crc = CRC16(&data[0], 39);
				data[39] = crc & 0xFF;
				data[40] = (crc >> 8) & 0xFF;

				len = 8;
				for (int i = 0; i < 41; i++)
					dataFrame.push_back(data[i]);
			}
			break;
			case HOME: // stop before send this command
			{
				data[0] = motorData.idMotor;
				data[1] = 0x06;
				data[2] = 0x00;
				data[3] = 0x7D;
				data[4] = 0;
				data[5] = 0x10;
				unsigned short crc = CRC16(&data[0], 6);
				data[6] = crc & 0xFF;
				data[7] = (crc >> 8) & 0xFF;

				for (int i = 0; i < 8; i++)
					dataFrame.push_back(data[i]);
			}
			break;
			case ANGLEOFFSET: // changed
			{
				int pulse = motorData.value * 36000 / 360;
				if (motorData.maxSpeed == 0)
					motorData.maxSpeed = 2000;
				int frequency = motorData.maxSpeed;

				data[0] = motorData.idMotor;
				data[1] = 0x10; // function code
				data[2] = 0x00; // register address
				data[3] = 0x58;
				data[4] = 0; // number of register 
				data[5] = 0x10;
				data[6] = 0x20; // bum of writing bytes
				data[7] = 0; // operation data number
				data[8] = 0;
				data[9] = 0;
				data[10] = 0;
				data[11] = 0; // operation type
				data[12] = 0;
				data[13] = 0;
				data[14] = 1; // NOSETTING = 0, ABSOLUTEPOSITIONING = 1, INCREMENTALPOSITIONINGKZD = 2/*based on position command*/, INCREMENTALPOSITIONINGKZD = 3/*based on position feedback*/ };//...more....
				data[15] = (pulse >> 24); // position
				data[16] = (pulse >> 16) & 0xFF;
				data[17] = (pulse >> 8) & 0xFF;
				data[18] = pulse & 0xFF;
				data[19] = (frequency >> 24); // operating speed (frequency)
				data[20] = (frequency >> 16) & 0xFF;
				data[21] = (frequency >> 8) & 0xFF;
				data[22] = frequency & 0xFF;

				data[23] = 00; // start/changing speed rate 1.5kHz/s
				data[24] = 00;
				data[25] = 0x05;
				data[26] = 0xDC;
				data[27] = 00; // stopping speed rate 1.5kHz/s
				data[28] = 00;
				data[29] = 0x05;
				data[30] = 0xDC;
				data[31] = 00; // operating current = 100%
				data[32] = 00;
				data[33] = 0x03;
				data[34] = 0xE8;
				data[35] = 00; // trigger all = 1 all date update
				data[36] = 00;
				data[37] = 00;
				data[38] = 0x1;
				unsigned short crc = CRC16(&data[0], 39);
				data[39] = crc & 0xFF;
				data[40] = (crc >> 8) & 0xFF;

				len = 8;
				for (int i = 0; i < 41; i++)
					dataFrame.push_back(data[i]);
			}
			break;
			}
		}
	}
	else return true;
}

/* This Function return Motor data from dataFrame that recieved from CAN BUS
Size of Data Frame:
Identifier: 0x140 + ID(1~32) (Steer Motor)
Frame Type: standard frame DLC: 8byte
	EX: Turn off Motor
	dataFrame[0] = 0x80; // command name
	dataFrame[1] = 0x00;
	dataFrame[2] = 0x00;
	dataFrame[3] = 0x00;
	dataFrame[4] = 0x00;
	dataFrame[5] = 0x00;
	dataFrame[6] = 0x00;
	dataFrame[7] = 0x00;
	dataFrame[8] = 0x08; size of frame data should be read. 13 byte, + 4 byte header + 1 byte end packet
	
	encodeFrame.id = id
	encodeFrame.motor_type = MOTOR_TYPE.STEER_MOTOR;
	encodeFrame.command_type = CONTROL_CMD::STOPMOTOR;
*/

bool MotorCmd::encodeFrameData(std::vector<byte>& dataFrame, MotorData & motorData)
{
	if (motorData.state_cmd == READ_CMD)
	{
		if (dataFrame[0] == 0x3E)
		{
			motorData.motor_type = STEER_MOTOR;
			switch (dataFrame[1])
			{
				case 0xA8: // response incremantel position command (INCREMENTALPOSITION) updated RS485
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					checkSum = 0;
					for (int i = 5; i < 12; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[12])
						return true;
					// finished checking data
					motorData.idMotor = dataFrame[2];
					unsigned char temperature = dataFrame[5];
					short Torque = dataFrame[6] + (dataFrame[7] << 8);
					int speedLimit = dataFrame[8] + (dataFrame[9] << 8) + (dataFrame[10] << 16) + (dataFrame[11] << 24);
				}
				break;
				case 0x88: // run motor, restore operation from stop
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
				}
				break;
				case 0x81: // stop motor
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
				}
				break;
				case 0x9C: // motor status
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					for (int i = 5; i < 12; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[12])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
					unsigned char temperature = dataFrame[5];
					short Torque = dataFrame[6] + (dataFrame[7] << 8);
					short speed = dataFrame[8] + (dataFrame[9] << 8);
					short encoder = dataFrame[10] + (dataFrame[11] << 8);

					motorData.value = speed;
				}
				break;
				case 0x19: // response set encoder as zero position
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
				}
				break;
				case 0xA2: // resoponse set speed motor
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					for (int i = 5; i < 11; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[12])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
					unsigned char tempeture = dataFrame[5];
					short torque = dataFrame[6] + (dataFrame[7] << 8);
					short speed = dataFrame[8] + (dataFrame[9] << 8);
					short encoder = dataFrame[10] + (dataFrame[11] << 8);

					motorData.value = speed;
				}
				break;
				case 0xA6: // response set single angle offset
				{
					unsigned char checkSum = 0;
					// re-check data
					for (int i = 0; i < 4; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[4])
						return true;
					for (int i = 5; i < 11; i++)
						checkSum += dataFrame[i];
					if (checkSum != dataFrame[12])
						return true;
					//finish checking data
					motorData.idMotor = dataFrame[2];
					unsigned char tempeture = dataFrame[5];
					short torque = dataFrame[6] + (dataFrame[7] << 8);
					short speed = dataFrame[8] + (dataFrame[9] << 8);
					short encoder = dataFrame[10] + (dataFrame[11] << 8);

					motorData.value = speed;
				}
				break;
			}
		}
		if ((dataFrame[0] == 0xFF) && (dataFrame[1] = 0xFE))
		{
			motorData.motor_type = DRIVE_MOTOR;
			switch (dataFrame[5])
			{
				case 0xD1: // position feedback
				{
					unsigned char checkSum = 0;
					for (int k = 0; k < 11; k++)
					{
						if (k != 0 && k != 1 && k != 4)
							checkSum += dataFrame[k];
					}
					if (checkSum != dataFrame[4])
						return true;
					motorData.idMotor = dataFrame[2];
					motorData.spinDirection = dataFrame[6];
					short location = (dataFrame[7] + (dataFrame[8] << 8))/100;
					short speed = (dataFrame[9] + (dataFrame[10] << 8))/10;
					unsigned char current = dataFrame[11] * 100; // mA
				}
				break;
				case 0xD7:
				{
					unsigned char checkSum = 0;
					for (int k = 0; k < 7; k++)
					{
						if (k != 0 && k != 1 && k != 4)
							checkSum += dataFrame[k];
					}
					if (checkSum != dataFrame[4])
						return true;
					motorData.idMotor = dataFrame[2];
					motorData.value = dataFrame[6];
				}
				break;
			}
		}
		else if (dataFrame[0] == 0xAA)
		{
			motorData.idMotor = dataFrame[4];
			if (((dataFrame[2]&0xFF) + ((dataFrame[3] << 8)&0xFF00)) > 0x600) // M4125
			{
				motorData.motor_type = M4215;
				motorData.idMotor = ((dataFrame[2]&0xFF) + ((dataFrame[3] << 8)&0xFF00)) - 0x600;
				switch ((dataFrame[5]&0xFF) + ((dataFrame[6] << 8)&0xFF00) + ((dataFrame[7] << 16)&0xFF0000))
				{
					case 0x607A:
					{
						if (dataFrame[4] == 0x60)
							return true; // wrote sucess
						else if (dataFrame[4] == 0x80)
							return false; // fail
					}
					break;
					case 0x6081:
					{
						if (dataFrame[4] == 0x60)
							return true; // wrote sucess
						else if (dataFrame[4] == 0x80)
							return false; // fail
					}
					break;
					case 0x6040:
					{
						if (dataFrame[4] == 0x60)
							return true; // wrote sucess
						else if (dataFrame[4] == 0x80)
							return false; // fail
					}
					break;
					case 0x6460:
					{
						if (dataFrame[4] == 0x43)
						{
							int value = (dataFrame[8] & 0xFF) + ((dataFrame[9] << 8) & 0xFF00) + ((dataFrame[10] << 16) & 0xFF0000) + ((dataFrame[11] << 24) & 0xFF000000);
							motorData.value = ((value*360) / 80) / 32768;
							return true; // wrote sucess
						}
						else
							return false; // fail
					}
					break;
				}

			}
			else
			{
				motorData.motor_type = M1010;
				switch (dataFrame[4 + 2])
				{
				case 0x16: // absolute position
				{
					motorData.value = dataFrame[8] | (dataFrame[9] << 8) | (dataFrame[10] << 16) | (dataFrame[11] << 24);
				}
				break;
				case 0x0c: //incremental position
				{
					motorData.value = dataFrame[8] | (dataFrame[9] << 8) | (dataFrame[10] << 16) | (dataFrame[11] << 24);
				}
				break;
				}
			}
		}
		else if (CRC16(&dataFrame[0], 6) == (dataFrame[6] +(dataFrame[7]<<8)))
		{
			motorData.motor_type = AZD_KD;
			motorData.idMotor = dataFrame[0];
		}
	}
	else
		return true;
}

//----------------------------------
#define BAURATE1000 0x01 // 1000k
// transfer frame
#define STD_FRAME 0x01
#define EXT_FRAME 0x02
//openration mode
#define NORMAL  0x00 //Normal
#define LOOPBACK 0x01 //Loopback
#define SILENT 0x02 //Silent(Listen - Only)
#define LOOPBACK_SILENT 0x03 //Loopback + Silent

void CANInitializationFrame(unsigned char *initializitionFrame) // send right after connecting COM port to establish CAN parameters, https://copperhilltech.com/blog/usbcan-analyzer-usb-to-can-bus-serial-protocol-definition/#:~:text=It%20allows%20you%20to%20develop,and%20analyzing%20CAN%20Bus%20data
{
	initializitionFrame[0] = 0xAA;
	initializitionFrame[1] = 0x55;
	initializitionFrame[2] = 0x12;
	initializitionFrame[3] = BAURATE1000;
	initializitionFrame[4] = STD_FRAME;
	initializitionFrame[5] = 0;
	initializitionFrame[6] = 0;
	initializitionFrame[7] = 0;
	initializitionFrame[8] = 0;
	initializitionFrame[9] = 0;
	initializitionFrame[10] = 0;
	initializitionFrame[11] = 0;
	initializitionFrame[12] = 0;
	initializitionFrame[13] = NORMAL;
	initializitionFrame[14] = 0x01;
	initializitionFrame[15] = 0;
	initializitionFrame[16] = 0;
	initializitionFrame[17] = 0;
	initializitionFrame[18] = 0;
	for (int i = 2; i <= 18; i++)
		initializitionFrame[19] = initializitionFrame[19] + initializitionFrame[i];
}
