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
//#define SWITCH_PIN 5
#define IR_PIN 7
#define BACK_IR_PIN 3
#define SIDE_IR_PIN 0
#define MAP_WIDTH 361
#define MAP_HEIGHT 361
#define TILE 27
#define NUM_PARTICLES 100

int state;
float head_Error, head_PreviousError;
float compassReading;
float wall_Error, wall_PreviousError;
double leftReading, midReading, rightReading;
float distanceFront;
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
	cout << "\nPink size: " << pink_size << endl;
}

void findOrange(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(220), orange_Gmax(245), orange_Bmin(185), orange_Bmax(205);
	//byte orange_Rmin(250), orange_Rmax(255), orange_Gmin(245), orange_Gmax(255), orange_Bmin(210), orange_Bmax(220);
	colorSegmenter(img, vec, height, width, orange_Rmin, orange_Rmax, orange_Gmin, orange_Gmax, orange_Bmin, orange_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(255,255,0), h_Error, orange_size, state, "orange", pink_size, orange_size, purple_size, green_size);
	cout << "\nOrange size: " << orange_size << endl;
}

void findPurple(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte purple_Rmin(225), purple_Rmax(255), purple_Gmin(120), purple_Gmax(200), purple_Bmin(165), purple_Bmax(255);
	colorSegmenter(img, vec, height, width, purple_Rmin, purple_Rmax, purple_Gmin, purple_Gmax, purple_Bmin, purple_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(128,0,128), h_Error, purple_size, state, "purple", pink_size, orange_size, purple_size, green_size);
	cout << "\nPurple size: " << purple_size << endl;
}

void findGreen(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width)
{
	byte green_Rmin(90), green_Rmax(115), green_Gmin(120), green_Gmax(160), green_Bmin(75), green_Bmax(110);
	colorSegmenter(img, vec, height, width, green_Rmin, green_Rmax, green_Gmin, green_Gmax, green_Bmin, green_Bmax);
	connectedComponents(vec, height, width);
	drawBoundingBox(img, height, width, vec, PixRGB<byte>(0,255,0), h_Error, green_size, state, "green", pink_size, orange_size, purple_size, green_size);
	cout << "\nGreen size: " << green_size << endl;
}

void updateProbability(std::vector<Particle> &particles, int direction, double distance, Image<PixRGB<byte> > mapImg)
{
	float sonar_variance = 5;
	double total_prob = 0;
	
  // update all the particle probabilities
	for (int i=0; i<particles.size(); i++)
	{
		// use heading to calculate the map distance from particle to wall
		Point2D<int> pos = (Point2D<int>)particles[i].getPosition();
		double map_distance =  distToEdge(pos, direction, mapImg);
		//std::cout << "Robot Position " << i << ": " << pos << std::endl;
		// std::cout << "Map Distance " << i << ": " << map_distance << std::endl;

		// Compute new probability using measured distance , map distance and sonar variance
		double new_prob = getProbability(map_distance, sonar_variance, distance);
		//std::cout << "getProb " << i << ": " << new_prob << std::endl;

		// update each probability and compute total probabilities
		double old_prob = particles[i].getWeight();
		//std::cout << "Old prob " << i << ": " << old_prob << std::endl;
		particles[i].setWeight(old_prob * new_prob);
		//std::cout << "Old_pro*new_prob " << i << ": " << particles[i].getWeight() << std::endl;
		total_prob += particles[i].getWeight();
		//std::cout << "Total prob " << i << ": " << total_prob << std::endl;
	}
  // Normalize all probabilities
	for (int i=0; i<particles.size(); i++)
	{
		particles[i].setWeight(particles[i].getWeight() / total_prob);
		//std::cout << "Final weight " << i << ": " << particles[i].getWeight() << std::endl;
	}
}


void sense(std::vector<Particle> &particles, int currentDirection, Image<PixRGB<byte> > mapImg){
	int left_direction, mid_direction, right_direction;
	switch(currentDirection) {
		case 0:	//north
			left_direction = 3;
			mid_direction = 0;
			right_direction = 2;
			break;
		case 1:	//south
			left_direction = 2;
			mid_direction = 1;
			right_direction = 3;
			break;
		case 2:	//east
			left_direction = 0;
			mid_direction = 2;
			right_direction = 1;
			break;
		case 3:	//west
			left_direction = 1;
			mid_direction = 3;
			right_direction = 0;
			break;
	}
	Nemo::setServo(SERVO_PIN, 165);	//Get Sonar Reading Left
	usleep(1000000);
    leftReading = getSonarMedian(5);
    if (leftReading < 150)
		updateProbability(particles, left_direction, leftReading, mapImg);	// Update Probability Left
	std::cout<< "Left Reading: " << leftReading << std::endl;

	Nemo::setServo(SERVO_PIN, 80);	//Get Sonar Reading Middle
	usleep(1000000);
    midReading = getSonarMedian(5);
	if (midReading < 150)
		updateProbability(particles, mid_direction, midReading, mapImg);	// Update Probability Middle
	std::cout<< "Middle Reading: " << midReading << std::endl;
	
	Nemo::setServo(SERVO_PIN, 0);	//Get Sonar Reading Right
	usleep(1000000);
    rightReading = getSonarMedian(5);
	if (rightReading < 150)
		updateProbability(particles, right_direction, rightReading, mapImg);// Update Probability Right
	std::cout<< "Right Reading: " << rightReading << std::endl;
	
	Nemo::setServo(SERVO_PIN, 80);
	particles = resampleParticles(particles);
}


int main()
{
	int RightMotorValue = 0;
	int LeftMotorValue = 0;
	h_PreviousError= 0;
	h_Error= 0;
	state = 0;
	
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
	//Nemo::setPullupAnalog(SWITCH_PIN, true);
	
	//INITIALIZE VISION
	Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	Nemo::openDisplayServerConnection(10005);
	Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
	Image<PixRGB<byte> > imgPink, imgOrange, imgPurple, imgGreen, particleMapImg;
	int width = img.getWidth();		//160px
	int height = img.getHeight();	//120px
	pink.resize(height, vector<int>(width));
	orange.resize(height, vector<int>(width));
	purple.resize(height, vector<int>(width));
	green.resize(height, vector<int>(width));
	
	//INITIALIZE PATH PLANNING
	Image<PixRGB<byte> > mapImg = Image<PixRGB<byte> >(Dims(MAP_WIDTH,MAP_HEIGHT), ZEROS);
	std::vector<Point2D<int> > edges, doors, docks, purpleDocks, orangeDoors;
	std::vector< vector<int> > purpleMap0, purpleMap1, purpleMap2, orangeMap0, orangeMap1, distmap, histmap;
	map13x13.resize(13, vector<int>(13));
	Point2D<int> initLoc, curLoc, goalLoc;
	int dock_index;
	char desiredDirection;
	
	int foundGreenFlag = 0;
	int exitOrangeFlag = 0;
	int dummy;
	
	//INITIALIZE PARTICLE FILTER
	double ang, dist, var, robot_x, robot_y, edgeDistance;
	Point2D<int> robot_pos, particleCurLoc;
	std::vector<Particle> particles;
	
	//INITIALIZE COMPASS and SONAR SERVO
	compassReading = getCompassMedian(10);
	if ( fabs(compassReading - 205) < 15)	{currentDirection = 0; ang = 270;}		//North
	else if ( fabs(compassReading - 327) < 15)	{currentDirection = 1; ang = 90;}	//South
	else if ( fabs(compassReading - 65) < 15)	{currentDirection = 2; ang = 0;}	//East
	else if ( fabs(compassReading - 278) < 15)	{currentDirection = 3; ang = 180;}	//West
	cout<<"Compass Reading:  "<<compassReading<<endl;
	cout << "Initial Direction: " << currentDirection << endl;
	Nemo::setServo(SERVO_PIN, 80);
	usleep(500000);
	
	//moveBackwardSonar(TILE, particles, ang, dist, var);

	
	while(1)
	{	
		cout << "\n\nState: " << state << endl;
		Image<PixRGB<byte> > img = Nemo::grabImage();
		
		switch(state) {
			case 0:		//search for robot, navigate map randomly
				h_PreviousError = h_Error;
				imgPink = img;
				findPink(imgPink,pink,height,width);
				imgOrange = img;
				findOrange(imgOrange,orange,height,width);

				distanceFront = getSonarMedian(5);
				cout << "Distance in front (SONAR): " << distanceFront << endl;
				
				if ( (orange_size > 400) && (orange_size < 19200) )
				{
					Nemo::setSpeaker(250);
					usleep(200000);
					Nemo::setSpeaker(0);
					orientNSEW('S', currentDirection, particles, ang, dist, var);	//turn around when you see a door
					imgOrange = Nemo::grabImage();		//get new images
					imgPink = Nemo::grabImage();
					usleep(1000000);
				}
				
				if (distanceFront < 28)	//if wall in front, stop and senseSonar
				{
					Nemo::setMotor(0, 0);
					Nemo::setMotor(1, 0);
					compassReading = getCompassMedian(10);
					if ( fabs(compassReading - 205) < 15)	currentDirection = 0;		//North
					else if ( fabs(compassReading - 327) < 15)	currentDirection = 1;	//South
					else if ( fabs(compassReading - 65) < 15)	currentDirection = 2;	//East
					else if ( fabs(compassReading - 278) < 15)	currentDirection = 3;	//West
					cout<<"Compass Reading:  "<<compassReading<<endl;
					cout << "Initial Direction: " << currentDirection << endl;
					senseSonar(leftReading, midReading, rightReading);
					if ( (leftReading<50) && (midReading<50) && (rightReading<50) )
					{
						if (currentDirection == 0)		orientNSEW('S', currentDirection, particles, ang, dist, var);
						else if (currentDirection == 1)	orientNSEW('N', currentDirection, particles, ang, dist, var);
						else if (currentDirection == 2)	orientNSEW('W', currentDirection, particles, ang, dist, var);
						else if (currentDirection == 3)	orientNSEW('E', currentDirection, particles, ang, dist, var);
					}
					else
						compassTurn90('R', currentDirection, leftReading, midReading, rightReading, particles, ang, dist, var);
				}
				
				followWallIR(myInterpolator, wall_Error, wall_PreviousError, IR_PIN);
				
				Nemo::displayImage(imgPink, 20);	//pink image
				break;
				
			case 1:	//found robot
				h_PreviousError = h_Error;
				imgPink = img;
				findPink(imgPink,pink,height,width);

				//PID
				if ( (abs(h_Error) > 5) )	//10degree deadband
				{
					horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
					usleep(200000);
					Nemo::setMotor(0, 0);
					Nemo::setMotor(1, 0);
				}
				moveForwardSonar(TILE/3, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				
				Nemo::displayImage(imgPink, 20);	//pink image
				break;
				
			case 2:	//very close to robot
				Nemo::setMotor(0, 0);
				Nemo::setMotor(1, 0);
				for (int i=0 ; i<3 ; i++)	//beep 3 times, play Mario level-up
				{
					Nemo::setSpeaker(750);
					usleep(200000);
					Nemo::setSpeaker(0);
					usleep(200000);
				}
				imgOrange = Nemo::grabImage();		//get new images
				imgPink = Nemo::grabImage();
				state = 3;
				
				Nemo::displayImage(imgPink, 20);	//pink image
				break;
				
			case 3:	//parse map.txt, initialize maps, and initialize particle filter
				for(int y=0; y<13; y++){	
					for(int x=0; x<13; x++)
						map13x13[y][x] = 0;}	//initialize map to 0
				parseFile(edges,doors,docks,purpleDocks,orangeDoors,initLoc,map13x13);	//add in walls
				drawMap(mapImg,edges,doors,docks,initLoc);	
				purpleMap0 = map13x13; purpleMap1 = map13x13; purpleMap2 = map13x13;
				orangeMap0 = map13x13; orangeMap1 = map13x13;
				
				// curLoc = initLoc/30;	
				curLoc.i = 7; curLoc.j = 11;
				particleCurLoc.i = curLoc.i*30; particleCurLoc.j = curLoc.j*30;
	
				for (int i=0 ; i<NUM_PARTICLES ; i++)	//create particles
				{
					Particle p = Particle(Point2D<int>(particleCurLoc.i-30+rand()%60,particleCurLoc.j-30+rand()%60), 1.0/NUM_PARTICLES);
					// Particle p = Particle(Point2D<int>(particleCurLoc.i,particleCurLoc.j), 1.0/NUM_PARTICLES);
					p.setWeight(double(1.0/NUM_PARTICLES));
					particles.push_back(p);
				}
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);

				desiredDirection = dirToDoor(curLoc, 1, orangeDoors, orangeMap0, orangeMap1);
				orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
				sense(particles, currentDirection, mapImg);
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				
				state = 4;
				Nemo::displayImage(particleMapImg, 20);
				break;
				
			case 4:	//escape from orange (go to orange and then +1)
				desiredDirection = dirToDoor(curLoc, 1, orangeDoors, orangeMap0, orangeMap1);
			//-----				
				if (desiredDirection == 'N')		dummy = 0;
				else if (desiredDirection == 'S')	dummy = 1;
				else if (desiredDirection == 'E')	dummy = 2;
				else if (desiredDirection == 'W')	dummy = 3;
				if (dummy != currentDirection)
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
					sense(particles, currentDirection, mapImg);
					particleMapImg = mapImg;
					particleMapImg = drawParticle(particleMapImg,particles);
					drawRobotPosition(particles, robot_pos, ang, particleMapImg);
					Nemo::displayImage(particleMapImg, 10);
				}
				else
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
				}
			//-----	
				moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				moveToDoor(curLoc, orangeDoors[1], desiredDirection, currentDirection, exitOrangeFlag, particles, ang, dist, var);	

				//subtract orange area from map, after you've escaped
				if (exitOrangeFlag == 1)
				{
					cout << "Exiting orange area..." << endl;
					orientNSEW('N', currentDirection, particles, ang, dist, var);	//escape from orange
					moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
					particleMapImg = mapImg;
					particleMapImg = drawParticle(particleMapImg,particles);
					drawRobotPosition(particles, robot_pos, ang, particleMapImg);
					Nemo::displayImage(particleMapImg, 10);
					curLoc.j = curLoc.j - 1;
					//set doors as walls to prevent from entering
					map13x13[orangeDoors[0].j][orangeDoors[0].i] = -2;
					map13x13[orangeDoors[1].j][orangeDoors[1].i] = -2;
					//map13x13[10][5] = -2; map13x13[7][6] = -2;
					map13x13[7][7] = -2; map13x13[8][7] = -2; map13x13[9][7] = -2; map13x13[10][7] = -2; map13x13[11][7] = -2;
					map13x13[11][5] = -2; map13x13[11][6] = -2; map13x13[9][8] = -2; map13x13[9][9] = -2; map13x13[10][9] = -2;
					map13x13[11][9] = -2; map13x13[11][10] = -2; map13x13[11][11] = -2; map13x13[10][11] = -2; map13x13[9][11] = -2;
					
					// for(int i = 0;i < doors.size()-1; i+=2){
						// Point2D<int> p1 = doors.at(i);
						// Point2D<int> p2 = doors.at(i+1);
						// drawLine(mapImg,p1,p2,PixRGB<byte>(0,250,250));	//white
					// }
					// drawMapNoDoors(mapImg,edges,doors,docks,initLoc);
					distmap = map13x13; histmap = map13x13;
					
					state = 5;
				}
				
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				cout << "Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				
				Nemo::displayImage(particleMapImg, 20);
				break;

			case 5: //look for green target (closest unvisited)
				imgGreen = img;
				findGreen(imgGreen,green,height,width);
				imgGreen = Nemo::grabImage();
				findGreen(imgGreen,green,height,width);
				imgGreen = Nemo::grabImage();
				findGreen(imgGreen,green,height,width);
				if ( (green_size > 200) && (green_size < 18000) )	//see green
				{
					state = 10;
					break;
				}
				
				cout << "Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				distmap = map13x13;
				goalLoc = findClosestUnvisitedOne(curLoc, distmap, histmap);
				cout << "Goal Location: (" << (int)goalLoc.i << ", " << (int)goalLoc.j << ")" << endl;
				distmap = map13x13;
				desiredDirection = dirToDummy(goalLoc, curLoc, distmap);
			//-----				
				if (desiredDirection == 'N')		dummy = 0;
				else if (desiredDirection == 'S')	dummy = 1;
				else if (desiredDirection == 'E')	dummy = 2;
				else if (desiredDirection == 'W')	dummy = 3;
				if (dummy != currentDirection)
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
					sense(particles, currentDirection, mapImg);
					particleMapImg = mapImg;
					particleMapImg = drawParticle(particleMapImg,particles);
					drawRobotPosition(particles, robot_pos, ang, particleMapImg);
					Nemo::displayImage(particleMapImg, 10);
				}
				else
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
				}
			//-----	
				moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				moveToGoal(curLoc, goalLoc, desiredDirection, currentDirection, histmap, particles, ang, dist, var);
			
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				
				Nemo::displayImage(particleMapImg, 20);
				break;
				
/*				
			case 4:	//look for green
				//put in "checkpoints" (desired coordinates where green is most likely to be), likely opposite sides of map
				//navigate map (keep distance from walls)
				//remember what coordinates you've already searched
				
				//take measurements and update particle filter
				//get out of orange section
				
				state = 5;
				break;
*/			
			
			case 6: //go to closest available purple dock
				desiredDirection = dirToDock(curLoc, dock_index, purpleDocks, purpleMap0, purpleMap1, purpleMap2);
				orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
				imgGreen = img;
				findGreen(imgGreen,green,height,width);
				imgGreen = Nemo::grabImage();
				findGreen(imgGreen,green,height,width);
				imgGreen = Nemo::grabImage();
				findGreen(imgGreen,green,height,width);
				if ( (green_size > 200) && (green_size < 18000) )	//see green
				{
					state = 10;
					break;
				}
				moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				moveToDock(curLoc, purpleDocks[dock_index], desiredDirection, currentDirection, histmap, state, particles, ang, dist, var);
				//look out for green, AVOID (do not run over)
				//look for purple
				//navigate map (keep distance from walls)
				//take measurements to update particle filter
				// purple_size = 0;
				// if ( (purple_size > 400) && (purple_size < 19200) )
					// state = 7;
				
				Nemo::displayImage(particleMapImg, 20);
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
				
			case 10:	//found green
				//h_PreviousError = h_Error;
				imgGreen = img;
				findGreen(imgGreen,green,height,width);
				imgGreen = Nemo::grabImage();
				findGreen(imgGreen,green,height,width);

				if (foundGreenFlag == 0)	//open arm
				{
					Nemo::setServo(GRAB_SERVO_PIN, 0);	//open
					usleep(2000000);
					foundGreenFlag = 1;
				}
				
				//PID
				if ( (abs(h_Error) > 5) )	//10degree deadband
				{
					horizPID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
					usleep(200000);
					Nemo::setMotor(0, 0);
					Nemo::setMotor(1, 0);
				}
				
				usleep(300000);
				moveForwardSonar(TILE/3, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				
				if ( (foundGreenFlag==1) && ((green_size>18000)||(green_size < 150)) )
				{
					state = 11;
				}
				
				Nemo::displayImage(particleMapImg, 20);
				break;

			case 11:	//grab green
				Nemo::setServo(GRAB_SERVO_PIN, 120);	//close
				usleep(2000000);
				for (int i=0 ; i<3 ; i++){Nemo::setSpeaker(600);usleep(150000);Nemo::setSpeaker(0);usleep(150000);}	//beep 3 times
				
				moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				if (currentDirection == 0)	orientNSEW('N', currentDirection, particles, ang, dist, var);
				else if (currentDirection == 1)	orientNSEW('S', currentDirection, particles, ang, dist, var);
				else if (currentDirection == 2)	orientNSEW('E', currentDirection, particles, ang, dist, var);
				else if (currentDirection == 3)	orientNSEW('W', currentDirection, particles, ang, dist, var);
				
				sense(particles, currentDirection, mapImg);
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				
				cout << "Robot Location: (" << (int)robot_pos.i << ", " << (int)robot_pos.j << ")" << endl;
				cout << "Previous Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				curLoc = getCurLoc(robot_pos);
				cout << "New Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				state = 12;
				
				Nemo::displayImage(particleMapImg, 20);
				break;
				
			case 12:	//bring green to dock
				desiredDirection = dirToDock(curLoc, dock_index, purpleDocks, purpleMap0, purpleMap1, purpleMap2);
			//-----				
				if (desiredDirection == 'N')		dummy = 0;
				else if (desiredDirection == 'S')	dummy = 1;
				else if (desiredDirection == 'E')	dummy = 2;
				else if (desiredDirection == 'W')	dummy = 3;
				if (dummy != currentDirection)
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
					sense(particles, currentDirection, mapImg);
					particleMapImg = mapImg;
					particleMapImg = drawParticle(particleMapImg,particles);
					drawRobotPosition(particles, robot_pos, ang, particleMapImg);
					Nemo::displayImage(particleMapImg, 10);
				}
				else
				{
					orientNSEW(desiredDirection, currentDirection, particles, ang, dist, var);
				}
			//-----	
				moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
				moveToDock(curLoc, purpleDocks[dock_index], desiredDirection, currentDirection, histmap, state, particles, ang, dist, var);
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				
				Nemo::displayImage(particleMapImg, 20);
				break;
			
			case 13:	//release green
				// release();
				Nemo::setServo(GRAB_SERVO_PIN, 0);	//open
				usleep(1500000);
				moveBackwardSonar(TILE*.5, particles, ang, dist, var);
				
				//turnAround
				// if (currentDirection == 0)		orientNSEW('S', currentDirection, particles, ang, dist, var);
				// else if (currentDirection == 1)	orientNSEW('N', currentDirection, particles, ang, dist, var);
				// else if (currentDirection == 2)	orientNSEW('W', currentDirection, particles, ang, dist, var);
				// else if (currentDirection == 3)	orientNSEW('E', currentDirection, particles, ang, dist, var);
				orientNSEW('E', currentDirection, particles, ang, dist, var);
				Nemo::setServo(GRAB_SERVO_PIN, 90);	//close
				usleep(1000000);
				cout << "\n\nReleased green object!\n\n";
				
				particleCurLoc.i = curLoc.i*30; particleCurLoc.j = curLoc.j*30;
				for (int i=0 ; i<NUM_PARTICLES ; i++)	//create particles
				{
					Particle p = Particle(Point2D<int>(particleCurLoc.i-30+rand()%60,particleCurLoc.j-30+rand()%60), 1.0/NUM_PARTICLES);
					p.setWeight(double(1.0/NUM_PARTICLES));
					particles.push_back(p);
				}
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				
				sense(particles, currentDirection, mapImg);
				particleMapImg = mapImg;
				particleMapImg = drawParticle(particleMapImg,particles);
				drawRobotPosition(particles, robot_pos, ang, particleMapImg);
				Nemo::displayImage(particleMapImg, 10);
				cout << "Robot Location: (" << (int)robot_pos.i << ", " << (int)robot_pos.j << ")" << endl;
				cout << "Previous Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				curLoc = getCurLoc(robot_pos);
				cout << "New Current Location: (" << (int)curLoc.i << ", " << (int)curLoc.j << ")" << endl;
				
				map13x13[purpleDocks[dock_index].j][purpleDocks[dock_index].i] = -2;
				purpleDocks[dock_index] = purpleDocks[dock_index+1];
				
				state = 5;
				
				Nemo::displayImage(particleMapImg, 20);
				break;
				
			default:
				cout << "Default." << endl;
				Nemo::setSpeaker(200);
		}
	
		//Nemo::displayImage(img, 20);	//normal image
		//Nemo::displayImage(imgPink, 20);	//pink image
		//Nemo::displayImage(imgOrange, 20);	//orange image
		// Nemo::displayImage(imgGreen, 20);	//green image
		// Nemo::displayImage(mapImg, 20);
		// Nemo::displayImage(particleMapImg, 20);
	}
	return 0;
}
