#include <iostream>
#include <nemo/Nemo.H>
#include <curses.h>
#include <Image/DrawOpsSimple.H>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <iostream>
#include <fstream>
#include <vision.h>
using namespace std;

//*****************************************************
#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7

int state;
int h_Error, f_Error, h_PreviousError, f_PreviousError;
int head_Error, head_PreviousError;
vector< vector<int> > pink;	//purple, green
double leftReading, midReading, rightReading;
float compassReading;
int currentDirection; //0N, 1S, 2E, 3W


Image<PixRGB<byte> > findPink(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte pink_Rmin(225), pink_Rmax(255), pink_Gmin(120), pink_Gmax(200), pink_Bmin(165), pink_Bmax(255);
	colorSegmenter(img, vec, height, width, pink_Rmin, pink_Rmax, pink_Gmin, pink_Gmax, pink_Bmin, pink_Bmax);
	connectedComponents(vec, height, width);
	int temp;
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(238,169,184), h_Error, f_Error, state);
	return img;
}

void orientNSEW(char dir, float &compassReading)
{
	int compassCommand;
	if (dir=='N')	compassCommand = 224;
	if (dir=='S')	compassCommand = 322;
	if (dir=='E')	compassCommand = 49;
	if (dir=='W')	compassCommand = 275;
	head_PreviousError = head_Error;
	head_Error = compassReading - compassCommand;
	
	while( (head_Error > 5) || (head_Error < -5) )
	{
		head_PreviousError = head_Error;
		head_Error = compassReading - compassCommand;
		float Kp(2), Ki(1), Kd(.5);
		float iError;
		iError += head_Error;
		int PIDControlValue = (Kp*head_Error) + (Ki*iError) + (Kd*(head_Error - head_PreviousError));
		int LeftMotorValue = PIDControlValue;
		int RightMotorValue = -PIDControlValue;
		if (LeftMotorValue > 100)		{LeftMotorValue = 100;}
		else if (LeftMotorValue < -100)		{LeftMotorValue = -1*100;}
		if (RightMotorValue > 100)		{RightMotorValue = 100;}
		else if (RightMotorValue < -100)		{RightMotorValue = -1*100;}
		Nemo::setMotor(0, LeftMotorValue);
		Nemo::setMotor(1, RightMotorValue);
		compassReading = Nemo::getCompass(2);
	}
	//usleep(600000);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void moveForward(float duration)
{
	usleep(500000);
	Nemo::setMotor(0, 70);
	Nemo::setMotor(1, 70);
	usleep(duration);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void senseSonar()
{
	Nemo::setServo(SERVO_PIN, 180);		// Get Sonar Reading Left
	usleep(1500000);//1.5 seconds
    leftReading = Nemo::getSonar(SONAR_PIN);
	std::cout<< "Left Reading: " << leftReading << std::endl;
	Nemo::setServo(SERVO_PIN, 85);		// Get Sonar Reading Middle
	usleep(1500000);//1.5 seconds
    midReading = Nemo::getSonar(SONAR_PIN);
	std::cout<< "Middle Reading: " << midReading << std::endl;
	Nemo::setServo(SERVO_PIN, 0);		// Get Sonar Reading Right
	usleep(1500000);//1.5 seconds
    rightReading = Nemo::getSonar(SONAR_PIN);
	std::cout<< "Right Reading: " << rightReading << std::endl;
	Nemo::setServo(SERVO_PIN, 85);		//reset servo to straight
}

int main()
{
	int RightMotorValue = 0;
	int LeftMotorValue = 0;
	h_Error=0; f_Error=0; h_PreviousError=0; f_PreviousError=0;
	state = 0;
	Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	Nemo::openDisplayServerConnection(10005);//Use 10001 you are team1 and 10002 for team2 and so on...

	compassReading = Nemo::getCompass(COMPASS_PIN);
	orientNSEW('E',compassReading);
	orientNSEW('E',compassReading);
	currentDirection = 2;
	
	while(1)
	{	
		Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
		int width = img.getWidth();		//160px
		int height = img.getHeight();	//120px
		pink.resize(height, vector<int>(width));
		//purple.resize(height, vector<int>(width));
		//green.resize(height, vector<int>(width));

		
		switch(state) {
			case 0:		//find injured pink robot to retrieve map
				senseSonar();
				if (currentDirection == 0)	//north
				{
					if (leftReading>midReading && leftReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('W', compassReading);
						orientNSEW('W', compassReading);
						moveForward(1500000);
					}
					else if (rightReading>leftReading && rightReading>midReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('E', compassReading);
						orientNSEW('E', compassReading);
						moveForward(1500000);
					}
					else if (midReading>leftReading && midReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('N', compassReading);
						orientNSEW('N', compassReading);
						moveForward(1500000);
					}
					else
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('S', compassReading);
						orientNSEW('S', compassReading);
						moveForward(1500000);
					}	
				}	
				if (currentDirection == 1)	//south
				{
					if (leftReading>midReading && leftReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('E', compassReading);
						orientNSEW('E', compassReading);
						moveForward(1500000);
					}
					else if (rightReading>leftReading && rightReading>midReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('W', compassReading);
						orientNSEW('W', compassReading);
						moveForward(1500000);
					}
					else if (midReading>leftReading && midReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('S', compassReading);
						orientNSEW('S', compassReading);
						moveForward(1500000);
					}
					else
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('N', compassReading);
						orientNSEW('N', compassReading);
						moveForward(1500000);
					}	
				}	
				
				if (currentDirection == 2)	//east
				{
					if (leftReading>midReading && leftReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('N', compassReading);
						orientNSEW('N', compassReading);
						moveForward(1500000);
					}
					else if (rightReading>leftReading && rightReading>midReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('S', compassReading);
						orientNSEW('S', compassReading);
						moveForward(1500000);
					}
					else if (midReading>leftReading && midReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('E', compassReading);
						orientNSEW('E', compassReading);
						moveForward(1500000);
					}
					else
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('W', compassReading);
						orientNSEW('W', compassReading);
						moveForward(1500000);
					}	
				}	
				
				if (currentDirection == 3)	//west
				{
					if (leftReading>midReading && leftReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('S', compassReading);
						orientNSEW('S', compassReading);
						moveForward(1500000);
					}
					else if (rightReading>leftReading && rightReading>midReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('N', compassReading);
						orientNSEW('N', compassReading);
						moveForward(1500000);
					}
					else if (midReading>leftReading && midReading>rightReading)
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('W', compassReading);
						orientNSEW('W', compassReading);
						moveForward(1500000);
					}
					else
					{
						compassReading = Nemo::getCompass(COMPASS_PIN);
						orientNSEW('E', compassReading);
						orientNSEW('E', compassReading);
						moveForward(1500000);
					}	
				}	
				
				// go straight if(midReading > 25.4)	//10in away from wall
				// turn left if( (midReading <= 25.4) && (leftReading > 49.96) && (rightReading <= 49.96) ) 
				// turn right if( (midReading <= 25.4 ) && (rightReading > 49.96) ) 
				// turn around if( (midReading <= 25.4) && (leftReading <= 49.96) && (rightReading <= 49.96) ) 
				// if reading>100, reading=25 //turn
				
				//turn randomly, turn around
				h_PreviousError = h_Error;
				f_PreviousError = f_Error;
				findPink(img,pink,height,width);
				
				// if ( ((h_Error > 10) || (h_Error < 10)) && (f_Error < 3.36) )	//10degree deadband for horiz && (object > img/5)
				// {
					// horizontal PID control ---> negative error, turn left
					// horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
					// Nemo::setMotor(0, LeftMotorValue);
					// Nemo::setMotor(1, RightMotorValue);
					// if ( (h_Error > 10) || (h_Error < -10) )
						// usleep(50000);

					// //forward PID control ---> negative error, slow down or back up
					// // forwardPID(LeftMotorValue, RightMotorValue, f_Error, f_PreviousError);
					// // Nemo::setMotor(0, LeftMotorValue);
					// // Nemo::setMotor(1, RightMotorValue);
				// }
				// if (image gets close enough to object, go straight to hit pink robot)
				// {
					// hitPinkRobot();
					// state = 1;
				// }
				break;
			case 1:
				Nemo::setMotor(0, 0);
				Nemo::setMotor(1, 0);
				Nemo::setSpeaker(440);
				usleep(500000);
				Nemo::setSpeaker(0);
				break;
			case 2:
				break;
			default:
				cout << "Default." << endl;
		}
	
		Nemo::displayImage(img, 20);//Display image to ilab3
	}
	return 0;
}
