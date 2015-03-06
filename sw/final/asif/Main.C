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
std::vector< vector<int> > map13x13;	//0: empty, 1: wall, 2: dock, 9: curLoc


void findPink(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte pink_Rmin(225), pink_Rmax(255), pink_Gmin(120), pink_Gmax(200), pink_Bmin(165), pink_Bmax(255);
	colorSegmenter(img, vec, height, width, pink_Rmin, pink_Rmax, pink_Gmin, pink_Gmax, pink_Bmin, pink_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(238,169,184), h_Error, pink_size, state, "pink", pink_size, orange_size, purple_size, green_size);
	cout << "Pink size: " << pink_size << endl;
}

void findOrange(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(220), orange_Gmax(245), orange_Bmin(185), orange_Bmax(205);
	//byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(245), orange_Gmax(255), orange_Bmin(210), orange_Bmax(220);
	colorSegmenter(img, vec, height, width, orange_Rmin, orange_Rmax, orange_Gmin, orange_Gmax, orange_Bmin, orange_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(255,255,0), h_Error, orange_size, state, "orange", pink_size, orange_size, purple_size, green_size);
	cout << "Orange size: " << orange_size << endl;
}

void findPurple(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte purple_Rmin(225), purple_Rmax(255), purple_Gmin(120), purple_Gmax(200), purple_Bmin(165), purple_Bmax(255);
	colorSegmenter(img, vec, height, width, purple_Rmin, purple_Rmax, purple_Gmin, purple_Gmax, purple_Bmin, purple_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(128,0,128), h_Error, purple_size, state, "purple", pink_size, orange_size, purple_size, green_size);
	cout << "purple size: " << purple_size << endl;
}

void findGreen(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte green_Rmin(115), green_Rmax(150), green_Gmin(140), green_Gmax(200), green_Bmin(100), green_Bmax(140);
	colorSegmenter(img, vec, height, width, green_Rmin, green_Rmax, green_Gmin, green_Gmax, green_Bmin, green_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(0,255,0), h_Error, green_size, state, "green", pink_size, orange_size, purple_size, green_size);
	cout << "green size: " << green_size << endl;
}


int main()
{
	int RightMotorValue = 0;
	int LeftMotorValue = 0;
	h_PreviousError= 0;
	h_Error= 0;
	state = 3;
	
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
	Nemo::setPullupAnalog(IR_PIN, true);
	Nemo::setPullupAnalog(SWITCH_PIN, true);
	
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
	
	Image<PixRGB<byte> > mapImg = Image<PixRGB<byte> >(Dims(MAP_WIDTH,MAP_HEIGHT), ZEROS);
	std::vector<Particle> particles;
	std::vector<Point2D<int> > edges, doors, docks, purpleDocks, ;
	std::vector< vector<int> > purpleMap0, purpleMap1, purpleMap2;
	map13x13.resize(13, vector<int>(13));
	Point2D<int> initLoc, curLoc;
	int dock_index;
	char desiredDirection;
	
	//orientNSEW('S', currentDirection);
	compassReading = getCompassMedian(10);
	if ( fabs(compassReading - 195) < 15)	currentDirection = 0;	//North
	else if ( fabs(compassReading - 312) < 15)	currentDirection = 1;	//South
	else if ( fabs(compassReading - 90) < 15)	currentDirection = 2;	//East
	else if ( fabs(compassReading - 269) < 15)	currentDirection = 3;	//West
	cout << "Initial Direction: " << currentDirection << endl;
	
	Nemo::setServo(SERVO_PIN, 80);
	usleep(500000);
	
	// while(1)
	// {
		// followWallIR(myInterpolator, wall_Error, wall_PreviousError);
		// cout << "Sonar: " << getSonarMedian(5) << endl << endl;
	// }

	while(1)
	{	
		Image<PixRGB<byte> > img = Nemo::grabImage();
		
		cout << "\n\nState: " << state << endl;
		
		switch(state) {
			case 0:		//search for robot, navigate map randomly
				h_PreviousError = h_Error;
				imgPink = img;	//make a copy
				findPink(imgPink,pink,height,width);
				imgOrange = img; //make a copy
				findOrange(imgOrange,orange,height,width);

				distanceIR = getIRMedian(myInterpolator, 7);
				cout << "Distance in front (IR): " << distanceIR << endl; //midReading << endl;
				
				if ( (orange_size > 400) && (orange_size < 19200) )
				{
					Nemo::setSpeaker(250);
					usleep(200000);
					Nemo::setSpeaker(0);
					orientNSEW('S', currentDirection);	//turn around when you see a door
					imgOrange = Nemo::grabImage();		//get new images
					imgPink = Nemo::grabImage();
					usleep(1000000);
				}
				
				if (distanceIR < 23)	//if wall in front, stop and senseSonar
				{
					Nemo::setMotor(0, 0);
					Nemo::setMotor(1, 0);
					senseSonar(leftReading, midReading, rightReading);
					if ( (leftReading<50) && (midReading<50) && (rightReading<50) )
					{
						if (currentDirection == 0)		orientNSEW('S', currentDirection);
						else if (currentDirection == 1)	orientNSEW('N', currentDirection);
						else if (currentDirection == 2)	orientNSEW('W', currentDirection);
						else if (currentDirection == 3)	orientNSEW('E', currentDirection);
					}
					else
						compassTurn90('R', currentDirection, leftReading, midReading, rightReading);
				}
				
				followWallSonar(wall_Error, wall_PreviousError, currentDirection, leftReading, midReading, rightReading);
				
				break;
				
			case 1:	//found robot
				h_PreviousError = h_Error;
				imgPink = img;
				findPink(imgPink,pink,height,width);
				
				if ( (abs(h_Error) > 10) && (pink_size > 150) )	//10degree deadband
				{
					horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
					if ( abs(h_Error) > 10 )
						usleep(50000);
				}
				moveForward(750000);	//move closer to pink robot
				break;
				
			case 2:	//very close to robot-->halt
				Nemo::setMotor(0, 0);
				Nemo::setMotor(1, 0);
				for (int i=0 ; i<3 ; i++)	//beep 3 times, play Mario level-up
				{
					Nemo::setSpeaker(750);
					usleep(200000);
					Nemo::setSpeaker(0);
					usleep(200000);
				}
				state = 3;
				break;
				
			case 3:	//parse map.txt and initialize particle filter
				for(int y=0; y<13; y++){	
					for(int x=0; x<13; x++)
						map13x13[y][x] = 0;}	//initialize map to 0
				parseFile(edges,doors,docks,purpleDocks,initLoc,map13x13);	//add in walls
				Nemo::displayImage(drawMap(mapImg,edges,doors,docks,initLoc), 10);	
				purpleMap0 = map13x13;
				purpleMap1 = map13x13;
				purpleMap2 = map13x13;
				
				curLoc = initLoc/30;
				// curLoc.i = 3;
				// curLoc.j = 1;
				
				cout << "Current Direction: " << currentDirection << endl;
				desiredDirection = dirToDock(curLoc, dock_index, purpleDocks, purpleMap0, purpleMap1, purpleMap2);
				//particleFilter();
				//state = 4;
				state = 6;
				break;
				
			case 4:	//look for green
				//put in "checkpoints" (desired coordinates where green is most likely to be), likely opposite sides of map
				//navigate map (keep distance from walls)
				//subtract orange area from map, after you've escaped
				//remember what coordinates you've already searched
				//take measurements to update particle filter
				state = 5;
				break;
			
			case 5: //found green, pick up green
				//update map (remove victim)
				state = 6;
				break;
			
			case 6: //go to closest available purple dock
				desiredDirection = dirToDock(curLoc, dock_index, purpleDocks, purpleMap0, purpleMap1, purpleMap2);
				moveToDock(curLoc, purpleDocks[dock_index], desiredDirection, currentDirection, state);
				//look for purple
				//navigate map (keep distance from walls)
				//take measurements to update particle filter
				purple_size = 0;
				if ( (purple_size > 400) && (purple_size < 19200) )
					state = 7;
				break;
			
			case 7: //found purple, release green on dock
				//remove dock from purpleDocks
				//update map, particle filter
				state = 4; //if more victims
				state = 8; //if finished
				break;
				
			case 8:
				//play fight song
				break;
				
			default:
				cout << "Default." << endl;
				Nemo::setSpeaker(200);
		}
	
		//Nemo::displayImage(img, 20);	//normal image
		//Nemo::displayImage(imgPink, 20);	//pink image
		//Nemo::displayImage(imgOrange, 20);	//orange image
	}
	return 0;
}
