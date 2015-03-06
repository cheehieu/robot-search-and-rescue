#ifndef PUTIL_H
#define PUTIL_H
#include <iostream>
#include <fstream>
#include <nemo/Nemo.H>
#include "Particle.H"
 
// Compute distance from point to line
//write a new distToEdge() to account for walls
//function to tell you what curPFLoc you are at (in cm)
//write a new function to translate between 361x361 PF map and 13x13 path planning map; if(curPFLoc>5 && curPFLoc<=25) curLoc=1; (account for robot size)
//come up with special cases, (sonar over long distances, fix itself if too close to wall (take another reading to be sure))
//given, current location and angle, return distance from right wall, middle wall, and left wall for particle filter
//iterate through drawn map using x,y coordinates to find closest walls (white pixel value)

double distToEdge(Point2D<int>pointLoc, int direction, Image<PixRGB<byte> > mapImg)
{
	int x = pointLoc.i;
	int y = pointLoc.j;
	double distance;
	byte curRed, curGreen, curBlue;

	while( 	((curRed != 255) && (curGreen != 255) && (curBlue != 255)) || 
			((curRed == 255) && (curGreen == 153) && (curBlue == 0)) ||
			((curRed == 204) && (curGreen == 0) && (curBlue == 204)) )	//while not white, or while orange/purple
	{
		if (direction == 0)			//north
		{
			x = x;
			y = y - 1;
			distance = abs(y - pointLoc.j);
		}
		else if (direction == 1)	//south
		{
			x = x;
			y = y + 1;
			distance = abs(y - pointLoc.j);
		}
		else if (direction == 2)	//east
		{
			x = x + 1;
			y = y;
			distance = abs(x - pointLoc.i);
		}
		else if (direction == 3)	//west
		{
			x = x - 1;
			y = y;
			distance = abs(x - pointLoc.i);
		}
		
		curRed = mapImg.getVal(x,y).red();
		curGreen = mapImg.getVal(x,y).green();
		curBlue = mapImg.getVal(x,y).blue();
	}

	return distance;
}

// double distToEdge(Point2D<double>point,int direction){
  // double distance;
  // if (direction == 0)
  // {
	// distance = point.j;
  // }
  // if (direction == 1)
  // {
	// distance = point.i;
  // }
  // if (direction == 2)
  // {
	// distance = 360 - point.j;
  // }
  // if (direction == 3)
  // {
	// distance = 360 - point.i;
  // }
  // return distance;
// }
// Compute probability given variance s,mean u
// #####################################################################
double getProbability(double u, double s, double x){
  double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);
  return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
}
// Draw particle on an image
// #####################################################################
Image<PixRGB<byte> > drawParticle(Image<PixRGB<byte> > &img, std::vector<Particle> &points){
 
  // for (int j=0; j<img.getHeight(); j++){
    // for (int i=0; i<img.getWidth(); i++){
      // img.setVal(i, j, PixRGB<byte>(255,255,255));
    // }
  // }
  for (int s=0; s<points.size(); s++){
    img.setVal(points[s].getPosition(), PixRGB<byte>(255,255,0));	//set yellow
  }
  return img;
}

void drawRobotPosition(std::vector<Particle> particles, Point2D<int> &robot_pos, double ang, Image<PixRGB<byte> > &particleMapImg)
{
	int radius = 7;
	double robot_x(0), robot_y(0);
	for (int i=0 ; i<particles.size() ; i++)
	{
		robot_x = robot_x + (double(particles[i].getPosition().i) * particles[i].getWeight());
		robot_y = robot_y + (double(particles[i].getPosition().j) * particles[i].getWeight());
	}
	robot_pos = Point2D<int>(int(robot_x),int(robot_y));
	drawCircle(particleMapImg,robot_pos,radius,PixRGB<byte>(255,0,0),3);	//red
	
	//draw robot direction
	Point2D<int> robot_dir = Point2D<int>(int(robot_x+radius*cos(ang)),int(robot_y+radius*sin(ang)));
	drawDisk(particleMapImg, Point2D<int>(robot_pos), radius/3, PixRGB<byte>(0,255,0));	//green
}


// #####################################################################
// Low variance sampler
// Use a roulette wheel to probabilistically duplicate particles with high weights,
// and discard those with low weights. A Particle is some structure that has
// a weight element w. The sum of all weights in oldParticles should equal 1.
std::vector<Particle> resampleParticles(std::vector<Particle> oldParticles){
  std::vector<Particle> newParticles;
  //Calculate a Cumulative Distribution Function for our particle weights
  std::vector<double> CDF;
  CDF.resize(oldParticles.size());
  CDF[0] = ((Particle)oldParticles[0]).getWeight();
  for(uint i=1; i<CDF.size(); i++)
    CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
  //Loop through our particles as if spinning a roulette wheel.
  //The random u that we start with determines the initial offset
  //and each particle will have a probability of getting landed on and
  //saved proportional to its posterior probability. If a particle has a very large
  //posterior, then it may get landed on many times. If a particle has a very low
  //posterior, then it may not get landed on at all. By incrementing by
  // 1/(numParticles) we ensure that we don't change the number of particles in our
  // returned set.
  uint i = 0;
  double u = randomDouble()* 1.0/double(oldParticles.size());
  for(uint j=0; j < oldParticles.size(); j++){
    while(u > CDF[i])
      i++;
    Particle p = oldParticles[i];
    p.setWeight(1.0/double(oldParticles.size()));
    newParticles.push_back(p);
    u += 1.0/double(oldParticles.size());
  }
  return newParticles;
}

Point2D<int> getCurLoc(Point2D<int> robot_pos)
{
	Point2D<int> currentLocation;
	int offset = 15;
	currentLocation.i = (robot_pos.i + offset) / 30;
	currentLocation.j = (robot_pos.j + offset) / 30;
	
	if (currentLocation.i == 0)
		currentLocation.i = 1;
	if (currentLocation.j == 0)
		currentLocation.j = 1;
	return currentLocation;
}



#endif