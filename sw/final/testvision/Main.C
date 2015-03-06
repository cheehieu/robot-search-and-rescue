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
#include <compass.h>
#include <grabber.H>
#include <irsense.h>
#include <map.h>
#include <math.h>
#include <motorcmd.h>
#include <Particle.H>
#include <sonar.h>
#include <Util.H>
#include <vision.h>
using namespace std;

//*****************************************************
#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define GRAB_SERVO_PIN 11
#define SWITCH_PIN 5
#define IR_PIN 7
#define SIDE_IR_PIN 0
#define BACK_IR_PIN 3
#define MAP_WIDTH 361
#define MAP_HEIGHT 361
#define TILE 27

int state;
float head_Error, head_PreviousError;
float compassReading;
float wall_Error, wall_PreviousError;
double leftReading, midReading, rightReading;
float distanceIR;
int h_Error, h_PreviousError;
vector< vector<int> > pink, orange, purple, green;
float pink_size, orange_size, purple_size, green_size;
int currentDirection;


void findPink(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte pink_Rmin(225), pink_Rmax(255), pink_Gmin(120), pink_Gmax(200), pink_Bmin(165), pink_Bmax(255);
	colorSegmenter(img, vec, height, width, pink_Rmin, pink_Rmax, pink_Gmin, pink_Gmax, pink_Bmin, pink_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(238,169,184), h_Error, pink_size, state, "pink", pink_size, orange_size, purple_size, green_size);
	cout << "Pink size: " << pink_size << endl;
}

Image<PixRGB<byte> > findOrange(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(220), orange_Gmax(245), orange_Bmin(185), orange_Bmax(205);
	//byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(245), orange_Gmax(255), orange_Bmin(210), orange_Bmax(220);
	colorSegmenter(img, vec, height, width, orange_Rmin, orange_Rmax, orange_Gmin, orange_Gmax, orange_Bmin, orange_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(238,169,184), h_Error, orange_size, state, "orange", pink_size, orange_size, purple_size, green_size);
	cout << "Orange size: " << orange_size << endl;
	return img;
}

void findGreen(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte green_Rmin(90), green_Rmax(115), green_Gmin(120), green_Gmax(160), green_Bmin(75), green_Bmax(110);
	colorSegmenter(img, vec, height, width, green_Rmin, green_Rmax, green_Gmin, green_Gmax, green_Bmin, green_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(0,255,0), h_Error, green_size, state, "green", pink_size, orange_size, purple_size, green_size);
	cout << "green size: " << green_size << endl;
}
	

int main()
{	
		//INITIALIZE IR SENSOR
	Interpolator myInterpolator;
	myInterpolator.insertDataPoint(2.75, 15);
	myInterpolator.insertDataPoint(2.55, 20);
	myInterpolator.insertDataPoint(2.00, 30);
	myInterpolator.insertDataPoint(1.55, 40);
	myInterpolator.insertDataPoint(1.25, 50);
	myInterpolator.insertDataPoint(1.07, 60);
	myInterpolator.insertDataPoint(0.90, 70);
	myInterpolator.insertDataPoint(0.80, 80);
	myInterpolator.insertDataPoint(0.70, 90);
	myInterpolator.insertDataPoint(0.65, 100);
	myInterpolator.insertDataPoint(0.6, 110);
	myInterpolator.insertDataPoint(0.55, 120);
	myInterpolator.insertDataPoint(0.5, 130);
	myInterpolator.insertDataPoint(0.48, 140);
	myInterpolator.insertDataPoint(0.45, 150);
	Nemo::setPullupAnalog(IR_PIN, true);
	Nemo::setPullupAnalog(SIDE_IR_PIN, true);
	Nemo::setPullupAnalog(BACK_IR_PIN, true);
	
	while(1)
	{
		// cout << "Back IR reading: " << getIRMedian(myInterpolator, 5, BACK_IR_PIN) << endl;
		// cout << "Sonar reading: " << getSonarMedian(5) << endl;
		cout << "Sonar reading: " << Nemo::getSonar(SONAR_PIN) << endl;
		usleep(100000);
	}
	
	
	// Nemo::setServo(SERVO_PIN, 80);		// middle
	// usleep(500000);
    // cout << "Sonar: " << getSonarMedian(5) << endl;
	// usleep(1000000);
	
	// moveFarForwardSonar( TILE, findFarthestSonar() );
	
	
/*	
	int LeftMotorValue(0), RightMotorValue(0);
	Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	Nemo::openDisplayServerConnection(10005);
	Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
	Image<PixRGB<byte> > imgPink, imgOrange, imgPurple, imgGreen;
	int width = img.getWidth();		//160px
	int height = img.getHeight();	//120px
	pink.resize(height, vector<int>(width));
	orange.resize(height, vector<int>(width));
	purple.resize(height, vector<int>(width));
	green.resize(height, vector<int>(width));
	
	while(1)
	{
		Image<PixRGB<byte> > img = Nemo::grabImage();
		imgGreen = img;
		findGreen(imgGreen,green,height,width);
		// if ( (green_size>150) && (green_size<18000) )
		// {
			// if ( (abs(h_Error) > 5) )	//10degree deadband
			// {
				// horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
					// usleep(100000);
			// }
			// Nemo::setMotor(0, 0);
			// Nemo::setMotor(1, 0);
		// }
		// usleep(500000);
		// moveForwardSonar(TILE/4, particles, ang, dist, var);
		Nemo::displayImage(imgGreen, 20);//Display image to ilab3
	}
*/	
	
	
	// float tile_distance = 27;
	// moveForwardSonar(tile_distance, particles, ang, dist, var);	//different tile_distance when carrying green
	
	// compassReading = getCompassMedian(10);
	// if ( fabs(compassReading - 195) < 15)	currentDirection = 0;	//North
	// else if ( fabs(compassReading - 312) < 15)	currentDirection = 1;	//South
	// else if ( fabs(compassReading - 90) < 15)	currentDirection = 2;	//East
	// else if ( fabs(compassReading - 269) < 15)	currentDirection = 3;	//West
	// cout << "Compass Reading: " << compassReading << endl;
	// cout << "Initial Direction: " << currentDirection << endl;
	// orientNSEW('S', currentDirection, particles, ang, dist, var);
	// compassReading = getCompassMedian(10);
	// cout << "Compass Reading: " << compassReading << endl;
	
	

//	MILESTONE 3 TESTING: NEED TO TUNE UP CAMERA GREEN DETECTION	
	// Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	// Nemo::openDisplayServerConnection(10005);
	// Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
	// Image<PixRGB<byte> > imgPink, imgOrange, imgPurple, imgGreen;
	// int width = img.getWidth();		//160px
	// int height = img.getHeight();	//120px
	// pink.resize(height, vector<int>(width));
	// orange.resize(height, vector<int>(width));
	// purple.resize(height, vector<int>(width));
	// green.resize(height, vector<int>(width));
	
	// compassReading = getCompassMedian(10);
	// if ( fabs(compassReading - 195) < 15)	currentDirection = 0;	//North
	// else if ( fabs(compassReading - 312) < 15)	currentDirection = 1;	//South
	// else if ( fabs(compassReading - 90) < 15)	currentDirection = 2;	//East
	// else if ( fabs(compassReading - 269) < 15)	currentDirection = 3;	//West
	// cout << "Compass Reading: " << compassReading << endl;
	// cout << "Initial Direction: " << currentDirection << endl;
	
	// int foundGreenFlag = 0;
	// int RightMotorValue = 0;
	// int LeftMotorValue = 0;

	// while(1)
	// {
		// Image<PixRGB<byte> > img = Nemo::grabImage();
		// imgGreen = img;
		// findGreen(imgGreen,green,height,width);
// /*		if ( (green_size>150) && (green_size<18000) )
		// {
			// if ( (abs(h_Error) > 10) )	//10degree deadband
			// {
				// horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
				// if ( abs(h_Error) > 10 )
					// usleep(50000);
			// }
			// if (foundGreenFlag == 0)
			// {
				// Nemo::setServo(GRAB_SERVO_PIN, 0);	//open
				// usleep(2000000);
			// }
			// moveForward(750000);
			// foundGreenFlag = 1;
		// }
		// if ( (foundGreenFlag==1) && (green_size>18000) ) //green_size<200 || 
		// {
			// moveForward(750000);
			// moveForward(750000);
			// while(Nemo::getADC(SWITCH_PIN) != 0)
			// {
				// cout << "Switch: " << Nemo::getADC(SWITCH_PIN) << endl;	
				// Nemo::setServo(GRAB_SERVO_PIN, 90);	//close
				// usleep(2000000);
			// }
			// usleep(5000000);
			// release(currentDirection);
		// }
// */		
		// Nemo::displayImage(imgGreen, 20);
	// }

/*
	Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	Nemo::openDisplayServerConnection(10005);
	Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
	Image<PixRGB<byte> > imgPink, imgOrange, imgPurple, imgGreen;
	int width = img.getWidth();		//160px
	int height = img.getHeight();	//120px
	pink.resize(height, vector<int>(width));
	orange.resize(height, vector<int>(width));
	purple.resize(height, vector<int>(width));
	green.resize(height, vector<int>(width));
	while(1)
	{	
		Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
		byte myred = img.getVal(80,90).red();
		byte mygreen = img.getVal(80,90).green();
		byte myblue = img.getVal(80,90).blue();
		cout << "R: " << int(myred) << '\t' << "G: " << int(mygreen) << '\t' << "B: " << int(myblue) << endl;
		drawLine(img, Point2D<int>(80,0), Point2D<int>(80,120), PixRGB<byte>(0,255,0), 1);	//green
		drawLine(img, Point2D<int>(0,90), Point2D<int>(160,90), PixRGB<byte>(0,255,0), 1);	//green
		usleep(250000);

		//imgOrange = img; //make a copy
		//findOrange(imgOrange,orange,height,width);
		
		// h_PreviousError = h_Error;
		// imgPink = img;	//make a copy
		// findPink(imgPink,pink,height,width);
		
		Nemo::displayImage(img, 20);//Display image to ilab3
	}
*/
	
	return 0;
}
