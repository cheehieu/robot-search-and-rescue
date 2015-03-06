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

 
// horizontal PID control
void horiz_PID(int &LeftMotorValue, int &RightMotorValue, int Error, int PreviousError) 
{
	float Kp(2), Ki(0), Kd(0);
	int PIDControlValue = Kp * Error + Kd * (Error - PreviousError);
	LeftMotorValue = PIDControlValue;
	RightMotorValue = -PIDControlValue;
	
	if (LeftMotorValue > 100)		LeftMotorValue = 100;
	if (LeftMotorValue < -100)		LeftMotorValue = -100;
	if (RightMotorValue > 100)		RightMotorValue = 100;
	if (RightMotorValue < -100)		RightMotorValue = -100;
}

// forward PID control
void forward_PID(int &LeftMotorValue, int &RightMotorValue, int Error, int PreviousError) 
{
	float Kp(15), Ki(0), Kd(0);
	int PIDControlValue = Kp * Error + Kd * (Error - PreviousError);
	LeftMotorValue = PIDControlValue;
	RightMotorValue = PIDControlValue;
	
	if (LeftMotorValue > 100)		LeftMotorValue = 100;
	if (LeftMotorValue < -100)		LeftMotorValue = -100;
	if (RightMotorValue > 100)		RightMotorValue = 100;
	if (RightMotorValue < -100)		RightMotorValue = -100;
}



int main()
{
	int RightMotorValue = 0;
	int LeftMotorValue = 0;
	int h_Error(0), f_Error(0), h_PreviousError, f_PreviousError;
  
	Nemo::startCamera(160, 120);//160x120 or 320x240 or 640x480
	Nemo::openDisplayServerConnection(10005);//Use 10001 you are team1 and 10002 for team2 and so on...
		
while(1)
{
	Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
	byte R_MIN(0), R_MAX(150), G_MIN(130), G_MAX(255), B_MIN(0), B_MAX(150);		
	byte myred, mygreen, myblue;
	
	int width = img.getWidth();		//160px
	int height = img.getHeight();	//120px
	vector< vector<int> > vec;
	vec.resize(height, vector<int>(width));
	
	for(int y=0; y<height; y++)
	{	
		for(int x=0; x<width; x++)
		{	
			myred = img.getVal(x,y).red();
			mygreen = img.getVal(x,y).green();
			myblue = img.getVal(x,y).blue();

			if ( myred>R_MIN && myred<R_MAX && 
				 mygreen>G_MIN && mygreen<G_MAX && 
				 myblue>B_MIN && myblue<B_MAX )
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
	
	
	// ***first pass using Raster Scanning Algorithm***
	int count(1), setID, min_label;
	int first_row(0), last_row(0), first_col(0), last_col(0);
	vector<int> neighbors;
	map<int, int> equiv_label;      //setID, min_label
	map<int,int>::iterator key;
	
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
	map<int, int> max_occur;      //setID, count
	
	for (int y=0 ; y<height ; y++)
	{
		for (int x=0 ; x<width ; x++)
		{
			if (vec[y][x] != 0)		// element is foreground
			{
				// relabel the element with the lowest equivalent label
				key = equiv_label.find(vec[y][x]);
				vec[y][x] = key->second;
				if (vec[y][x] > 0)	//find biggest object by occurrance
					max_occur[ vec[y][x] ]++;		//does this really work?
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
	
	// find border of image
	int row_max(0), row_min(width), col_max(0), col_min(height);
	for(int y=0 ; y<height ; y++)
	{
		for(int x=0 ; x<width ; x++)
		{
		if (vec[y][x] == max_key)
		{
			img.setVal(x, y, PixRGB<byte>(255,0,0));
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
	
	// locate center and horizontal error
	int img_center = width/2;
	drawLine(img, Point2D<int>(img_center,0), Point2D<int>(img_center,height), PixRGB<byte>(0,255,0), 1);	//green
	int obj_vcenter = (col_max - col_min)/2 + col_min;
	int obj_hcenter = (row_max - row_min)/2 + row_min;
	drawDisk(img, Point2D<int>(obj_hcenter,obj_vcenter), 5, PixRGB<byte>(200,200,100));	//some color

	// horizontal PID control ---> negative error, turn left
	h_PreviousError = h_Error;
	h_Error = obj_hcenter - img_center;
	horiz_PID(LeftMotorValue, RightMotorValue, h_Error, h_PreviousError);
	Nemo::setMotor(0, LeftMotorValue);
	Nemo::setMotor(1, RightMotorValue);
	if ( (h_Error > 10) || (h_Error < -10) )
		usleep(50000);

	// forward PID control ---> negative error, slow down or back up
	float calib_obj_size = (width/2) * (height/2);		// a fourth of the image
	float obj_size = (row_max - row_min) * (col_max - col_min);
	f_PreviousError = f_Error;
	f_Error = (calib_obj_size - obj_size) / (calib_obj_size/4);		// normalize
	forward_PID(LeftMotorValue, RightMotorValue, f_Error, f_PreviousError);
	Nemo::setMotor(0, LeftMotorValue);
	Nemo::setMotor(1, RightMotorValue);
	
	Nemo::displayImage(img, 20);//Display image to ilab3
 
	}
	return 0;
}
