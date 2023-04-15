#include "PC_FileIO.c"
#include "EV3Servo-lib-UW.c"
#include "mindsensors-motormux.h"

void drive_dist(int motor_power, int distance);
void drive(int motor_power, int tetrix_1, int tetrix_2);
void sensorconfig();
void rotate_robot(int angle);
void drive_both(int motor_power_C, int motor_power_D);
void drive_to_destination(int stairs);
void drive_to_origin(int stairs);
bool detect_color();
void file_read(TFileHandle &FileIn, string &name, int *password, int &stairs);
void door();
bool wait_for_customer();
int read_button( );
bool check_password(int *password);
const float CM_TO_DEG = 180/(2.75*PI);


int password[4];


task main()
{
 sensorconfig();

	TFileHandle FileIn;
	bool fileokay = openReadPC(FileIn, "s_txt.txt");
	if(!fileokay)
		{
			displayString(5, "Error opening the file!");
			wait1Msec(5000);
		}
	drive(0,1,2);
	string name = "";
	int stairs=0;

	setServoPosition(S1,3,27);
	setServoPosition(S1,4,70);
	wait1Msec(2000);

	file_read(FileIn, name, password, stairs);

	drive_to_destination(stairs);

	bool return_to_original = detect_color();
	if(return_to_original)
	{
		drive_to_origin(stairs);
	}
	else
	{
		displayString(5, "blue");
		wait1Msec(5000);
		bool interaction = wait_for_customer();
		if(interaction)
		{
			bool valid_customer = check_password(password);
			if(valid_customer)
			{
				door();
				eraseDisplay();
				displayString(5, "Thank you for having no");
				displayString(6, "legs to help our business");
				wait1Msec(5000);
				drive_to_origin(stairs);
			}
			else
			{
				eraseDisplay();
				displayString(5, "Stop stealing other");
				displayString(6, "people's food");
				wait1Msec(5000);
				drive_to_origin(stairs);
			}
		}
		else
		{
			drive_to_origin(stairs);
		}
	}

	closeFilePC(FileIn);
}

void file_read(TFileHandle &FileIn, string &name, int *password, int &stairs)
{
	readTextPC(FileIn, name);
	for(int count = 0; count < 4; count++)
	{
		int temp = 0;
		readIntPC(FileIn, temp);
		password[count] = temp;
	}

	readIntPC(FileIn, stairs);
}


void sensorconfig()
{
	SensorType[S4] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Color_Color;
	wait1Msec(50);
	SensorType[S3] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S3] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[S3] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	SensorType[S2] = sensorI2CCustom9V;
	wait1Msec(50);
	SensorType[S2] = sensorI2CCustom;
	MSMMUXinit();
	wait1Msec(50);
}

void drive_dist(int motor_power, int distance)
{
	nMotorEncoder[motorA] = 0;
	motor[motorD] = motor[motorC] = motor_power;
	motor[motorB] = motor[motorA] = -motor_power;
	while(nMotorEncoder[motorA] < distance*CM_TO_DEG)
	{}
	motor[motorA] = motor[motorD] = motor[motorC] = motor[motorB] = 0;
}
void tetrix_move(int power_tetrix_1, int power_tetrix_2)
{
	setServoSpeed(S1,1,power_tetrix_1,-17,-4);
	setServoSpeed(S1,2,power_tetrix_2,-23,-13);
}
void drive(int motor_power, int tetrix_1, int tetrix_2)
{
	tetrix_move(tetrix_1, tetrix_2);
	motor[motorD] = motor[motorC] = motor_power;
	motor[motorB] = motor[motorA] = -motor_power;
}

void drive_both(int motor_power_C, int motor_power_D)
{
    motor[motorC] = motor_power_C;
    motor[motorD] = motor_power_D;
}
void rotate_robot(int angle)//Positive angles are clockwise when viewed from above
{
		setServoPosition(S1,3,angle);
		setServoPosition(S1,4,angle);
    int angle_total=0;
    angle_total=SensorValue[S3]+angle;
    if (angle>0)
    {
        drive_both(-5,30);
        while (getGyroDegrees(S3)<angle_total)
        {}
    }
    else
    {
        drive_both(30,-5);
        while (getGyroDegrees(S3)>angle_total)
        {}
    }
    drive(0,1,2);
}


void drive_to_destination(int stairs)
{
	drive(80, 90, -90);
	int count_step = 0;
	while(count_step < stairs)
	{
		drive(80, 90, -90);
		while(SensorValue[S3] > -5 && SensorValue[S3] < 5)
		{}
		wait1Msec(2000);
		while(abs(SensorValue[S3]) > 5)
		{}
		wait1Msec(800);
		count_step++;
	}
	wait1Msec(500);
	drive(0, 1, 2);
	//displayString(5, "%d", count_step);
	//wait1Msec(5000);
}

void drive_to_origin(int stairs)
{
	drive(40, 60, -60);
	int count_step = 0;
	while(count_step < stairs)
	{
		drive(40, 60, -60);
		while(SensorValue[S3] > -5 && SensorValue[S3] < 5)
		{}
		wait1Msec(2000);
		while(abs(SensorValue[S3]) > 5)
		{}
		wait1Msec(800);
		count_step++;
	}
	wait1Msec(1000);
	drive(0, 1, 2);
	//displayString(5, "%d", count_step);
	//wait1Msec(5000);
}


bool detect_color()
{
	drive(40, 60, -60);
	wait1Msec(2000);
	while(SensorValue[S4]!=(int)colorBlue && SensorValue[S4] != (int)colorRed)
	{}
	drive(0,1,2);
	wait1Msec(2000);
	bool status = false;
	if(SensorValue[S4] == (int)colorBlue)
	{
		status = false;
	}
	else if(SensorValue[S4] == (int)colorRed)
	{
		status = true;
	}
	return status;
}


bool wait_for_customer()
{
	time1[T1] = 0;
	bool customer_status=true;
  eraseDisplay();
	displayString(5, "Please Press the Enter button");
	while(!getButtonPress(buttonEnter) && time1[T1]<10000)
	{}
	int time=time1[T1];
	if(time>=10000)
		customer_status=false;
	return customer_status;
}


void door()
{
	MSMMotor(mmotor_S2_1, 20);
	wait1Msec(1000);
	MSMotorStop(mmotor_S2_1);
	time1[T1] = 0;
	eraseDisplay();
	displayString(5, "Please take your parcel")
	while(time1[T1] < 20000 && !getButtonPress(buttonEnter))
	{}
	MSMMotor(mmotor_S2_1, -20);
	wait1Msec(1000);
	MSMotorStop(mmotor_S2_1);
}

int read_button()
{
	int button_signal= -1;
	if(getButtonPress(buttonEnter)==1)
	button_signal=0;
	else if (getButtonPress(buttonUp)==1)
	button_signal=1;
	else if (getButtonPress(buttonLeft)==1)
	button_signal=2;
	else if (getButtonPress(buttonDown)==1)
	button_signal=3;
	else if (getButtonPress(buttonRight)==1)
	button_signal=4;
	return button_signal;
}

bool check_password(int *password)
{
	eraseDisplay();
	displayString(5, "Please enter correct password");
	wait1Msec(5000);
	bool password_correct = false;
	int password_temp[4] ;
	for(int index = 0; index < 4; index++)
	{
		while (!getButtonPress(buttonAny))
	{}
		password_temp[index] = read_button();
		eraseDisplay();
		displayString(5,"%d", password_temp[index]);
		wait1Msec(2000);
	}
	if(password[0] == password_temp[0] && password[1] == password_temp[1] && password[2] == password_temp[2] && password[3] == password_temp[3])
	{
		password_correct = true	;
		displayString(5, "Correct");
		wait1Msec(5000);
	}
	else
	{
		password_correct = false;
		displayString(5, "Wrong");
		wait1Msec(5000);
	}
	return password_correct;
}
