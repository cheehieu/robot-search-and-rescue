#ifndef PARTICLE_H
#define PARTICLE_H
 
#include <cmath>
#include <nemo/Nemo.H>
 
//#####################################################################
double randomDouble()
{
  return double(rand()) / (double(RAND_MAX) + 1.0);
}
// #####################################################################
double randomDoubleFromNormal(const double s)
{
  double sum = 0;
  for(int i=0; i<12; i++){
    sum += randomDouble()*2*s - s;
  }
  return sum/2;
}
// #####################################################################
class Particle{
 
  public:
 
    Particle(Point2D<int> start = Point2D<int>(0,0), double weight = 0)
    {
      coord = start;
      weight = 0;
    }
    ~Particle(){}
 
    // Moves the particle position with noise added to angle and distance
    void moveParticle(double angle, double distance, double variance){
      double realAngle = angle + randomDoubleFromNormal(variance);
      double realDistance = distance + randomDoubleFromNormal(variance);
	  
	  //account for particles moving out of map boundaries
	  double bound_n = 0;	//north wall
	  double bound_s = 360;	//south wall
	  double bound_e = 360;	//east wall
	  double bound_w = 0;	//west wall
	  double temp_i = int(double(coord.i) + realDistance* cos(realAngle * M_PI/180.0));
	  double temp_j = int(double(coord.j) + realDistance* sin(realAngle * M_PI/180.0));
	  coord.i = (temp_i < bound_w+1) ? bound_w+1 : temp_i;
	  coord.i = (temp_i > bound_e-1) ? bound_e-1 : coord.i;
	  coord.j = (temp_j < bound_n+1) ? bound_n+1 : temp_j;
	  coord.j = (temp_j > bound_s-1) ? bound_s-1 : coord.j;
    }
 
    Point2D<int> getPosition(){return coord;}
    // Get Particle's probability
    double getWeight(){return weight;}
    // Modifies the Particle's probability
    void setWeight(double w){weight = w;}
 
  private:
    Point2D<int> coord; // coordinates of this particle
    double weight;  // probability that this particle represents correct location
 
};
#endif