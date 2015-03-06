#ifndef VISION_H
#define VISION_H

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
using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7

// white = PixRGB<byte>(255,255,255);
// black = PixRGB<byte>(0,0,0);
// red = PixRGB<byte>(255,0,0);
// green = PixRGB<byte>(0,255,0);
// blue = PixRGB<byte>(0,0,255);
// pink = PixRGB<byte>(238,169,184);
// purple = PixRGB<byte>(128,0,128);
// yellow = PixRGB<byte>(255,255,0);


void colorSegmenter(Image<PixRGB<byte> > &img, vector<vector<int> > &vec, int height, int width, byte Rmin, byte Rmax, byte Gmin, byte Gmax, byte Bmin, byte Bmax)
{
	for(int y=0; y<img.getHeight(); y++)
	{	
		for(int x=0; x<img.getWidth(); x++)
		{	
			byte myred = img.getVal(x,y).red();
			byte mygreen = img.getVal(x,y).green();
			byte myblue = img.getVal(x,y).blue();
			if ( myred>=Rmin && myred<=Rmax && 
				 mygreen>=Gmin && mygreen<=Gmax && 
				 myblue>=Bmin && myblue<=Bmax )
			{
				img.setVal(x, y, PixRGB<byte>(255,255,255));	// set pixel to white
				vec[y][x] = 1;
			}
			else
			{
				img.setVal(x, y, PixRGB<byte>(0,0,0));	// set pixel to black
				vec[y][x] = 0;
			}
		}
	}
}


void connectedComponents(vector<vector<int> > &vec, int height, int width)
{
	int count(1), setID, min_label;
	int first_row(0), last_row(0), first_col(0), last_col(0);
	vector<int> neighbors;
	map<int, int> equiv_label;	//setID, min_label
	map<int,int>::iterator key;
	
	// ***first pass using Raster Scanning Algorithm***
	// iterate through each element of the data by column, then by row 
	for (int y=0 ; y<height ; y++)
	{
		for (int x=0 ; x<width ; x++)
		{
			if (vec[y][x] != 0)	// if element is foreground
			{
				// accounting for special cases when there are < 8 neighbors
				first_row = (y==0) ? 1 : 0;
				last_row = (y==height-1) ? 1 : 0;
				first_col = (x==0) ? 1 : 0;
				last_col = (x==width-1) ? 1 : 0;
				
				// find neighbors NW N, NE, and W of current element
				neighbors.clear();
				if (first_row==1 && first_col==1)
				{
					neighbors.push_back (0);
					neighbors.push_back (0);
					neighbors.push_back (0);
					neighbors.push_back (0);
				}
				else if (first_row==1 && first_col==0)
				{
					neighbors.push_back (0);
					neighbors.push_back (0);
					neighbors.push_back (0);
					neighbors.push_back (vec[y][x-1]);
				}
				else if (first_row==0 && first_col==1)
				{
					neighbors.push_back (0);
					neighbors.push_back (0);
					neighbors.push_back (vec[y-1][x]);
					neighbors.push_back (vec[y-1][x+1]);
				}
				else
				{
					neighbors.push_back (vec[y-1+first_row][x-1+first_col]);
					neighbors.push_back (vec[y-1+first_row][x]);
					neighbors.push_back (vec[y-1+first_row][x+1-last_col]);
					neighbors.push_back (vec[y][x-1+first_col]);
				}
		
				// if no neighbors > 0, uniquely label current element
				if (*max_element(neighbors.begin(),neighbors.end()) == 0)
				{
					vec[y][x] = count;
					equiv_label[vec[y][x]] = vec[y][x];
					count++;
				}
				// label current element with smallest neighbor > 0
				else
				{
					vec[y][x] = *max_element(neighbors.begin(),neighbors.end());
					for (int i=0 ; i<neighbors.size() ; i++)
					{
						if ( (neighbors[i] < vec[y][x]) && (neighbors[i] != 0) )
							vec[y][x] = neighbors[i];
					}
				}
				
				// store the equivalence between neighboring labels                                                   
				min_label = *max_element(neighbors.begin(),neighbors.end());
				for (int i=0 ; i<neighbors.size() ; i++)
				{
					if ( (neighbors[i] < min_label) && (neighbors[i] != 0) )
					{
						min_label = neighbors[i];
						key = equiv_label.find(vec[y][x]);
						if (min_label < key->second)
						equiv_label[ vec[y][x] ] = min_label;
					}
					if ((neighbors[i] != 0))
					{
						key = equiv_label.find(neighbors[i]);
						if ((key->second == 0))
							equiv_label[neighbors[i]] = vec[y][x];
						else if(vec[y][x] < key->second)
						{
							key = equiv_label.find(vec[y][x]);
							equiv_label[neighbors[i]] = key->second;
						}
					}
				}					
			}
		}
	}
	
	// *** second pass using Raster Scanning Algorithm ***
	// iterate through each element of the data by column, then by row
	for (int y=0 ; y<height ; y++)
	{
		for (int x=0 ; x<width ; x++)
		{
			if (vec[y][x] != 0)		// element is foreground
			{
				// relabel the element with the lowest equivalent label
				key = equiv_label.find(vec[y][x]);
				vec[y][x] = key->second;
			}
		}
	}
}


// ##### draw bounding box around biggest color-segmented object in the image #####
void drawBoundingBox(Image<PixRGB<byte> > &img, int height, int width, vector<vector<int> > &vec, PixRGB<byte> obj_color, 
					int &h_Error, float &obj_size, int &state, string id, float &pk_size, float &or_size, float &pp_size, float &gn_size)
{
	// find biggest connected component by occurrance
	map<int, int> max_occur;      //setID, count
	for (int y=0 ; y<height ; y++)
	{
		for (int x=0 ; x<width ; x++)
		{
			if (vec[y][x] != 0)		//element is foreground
			{
				if (vec[y][x] > 0)	//increment occurrance
					max_occur[ vec[y][x] ]++;
			}
		}
	}
	int max_value = 0, max_key = 0;
	for(map<int,int>::iterator key=max_occur.begin() ; key!=max_occur.end() ; key++)
	{
		if (key->second > max_value)
		{
			max_value = key->second;
			max_key = key->first;
		}
	}
	
	// find border of biggest connected component
	int row_max(0), row_min(width), col_max(0), col_min(height);
	for(int y=0 ; y<height ; y++)
	{
		for(int x=0 ; x<width ; x++)
		{
			if ( (vec[y][x]==max_key) && (max_key!=0) )
			{
				img.setVal(x, y, obj_color);	//set object color
				if (x>=row_max)		row_max = x;
				if (x<row_min)		row_min = x;
				if (y>=col_max)		col_max = y;
				if (y<col_min)		col_min = y;
			}
		}
	}

	// draw bounding box around object
	Point2D<int> TLbound_box, BRbound_box;
	TLbound_box = Point2D<int>(row_min,col_min);
	BRbound_box = Point2D<int>(row_max,col_max);
	drawRect(img, TLbound_box, BRbound_box, PixRGB<byte>(0,0,255), 1);	//blue
	
	// draw center of bounding box
	int horiz_center = width/2;
	int vert_center = height/2;
	int obj_vcenter = (col_max - col_min)/2 + col_min;
	int obj_hcenter = (row_max - row_min)/2 + row_min;
	drawDisk(img, Point2D<int>(obj_hcenter,obj_vcenter), 3, PixRGB<byte>(255,255,0));	//yellow
	
	// calculate center errors for PID
	if (id!="orange")
		h_Error = obj_hcenter - horiz_center;
	obj_size = (row_max - row_min) * (col_max - col_min);
	
	if (id=="pink")
	{
		if (obj_size <= 125 || obj_size == 19200)
			state = 0;	//did not find object
		if ( (obj_size > 125) && (obj_size < 19200) && (or_size == 19200) ) //see only pink, no orange
		{	
			cout << "Entering State 1..." << endl;
			state = 1;	//found object > 2.5%*imagesize	
		}
		if ( (obj_size > 1500) && (obj_size < 19200) )
		{
			cout << "Found pink robot!" << endl;
			state = 2;	//very close. object>80%imagesize
		}
	}	
}

// ##### horizontal PID control #####
void horizPID(int &LeftMotorValue, int &RightMotorValue, int Error, int PreviousError) 
{
	float Kp(1.5), Ki(0), Kd(0);
	int PIDControlValue = Kp * Error + Kd * (Error - PreviousError);
	LeftMotorValue = PIDControlValue;
	RightMotorValue = -PIDControlValue;
	if (LeftMotorValue > 100)		LeftMotorValue = 100;
	if (LeftMotorValue < -100)		LeftMotorValue = -100;
	if (RightMotorValue > 100)		RightMotorValue = 100;
	if (RightMotorValue < -100)		RightMotorValue = -100;
	Nemo::setMotor(0, LeftMotorValue);
	Nemo::setMotor(1, RightMotorValue);
}

#endif
