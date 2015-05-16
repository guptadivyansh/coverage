#include <iostream>
#include <cstdlib>
#include <cmath>
#include <queue>

#include "Coverage.h"
#include "VoronoiDiagramGenerator.h"

using namespace cv;
using namespace std;

Coverage::~Coverage()
{
	map.release();
}

Point Coverage::GetNearestFreeCentroid(Point centre)
{
	//Will push centroid to nearest free location
	//BFS (8-connected) used to get nearest point
	int k,l,r=map.rows,c=map.cols;
	vector< vector<bool> > visited(r,vector<bool>(c,false));
	queue<Point> frontier;

	frontier.push(centre);
	visited[centre.y][centre.x] = true;

	Point result;

	while(!frontier.empty()){
		Point top = frontier.front();
		frontier.pop();

		bool flag=false;
		for(k=top.y-1;k<=top.y+1;k++){
			for(l=top.x-1;l<=top.x+1;l++){
				if(IsValidPoint(l,k,r,c)==false||visited[k][l]==true)
						continue;
				if(map.at<uchar>(k,l)==255){
					//Got the nearest free point
					result.y=k; result.x=l;
					return result;
				}
				Point nbr(l,k);
				visited[k][l] = true;
				frontier.push(nbr);
			}
		}
	}

	return centre; //Will return centre

}

void Coverage::DisplayPoints(vector<Point>& locations)
{
	//Will display the map with locations desired (coloured points)

	Mat col_copy(map.size(),CV_8UC3);//Will store a 3-channel copy of map
	int i,j,r=map.rows,c=map.cols;

	//Create 3-channel copy of map
	for(i=0;i<r;i++){
		for(j=0;j<c;j++){
			col_copy.at<Vec3b>(i,j)[0] = col_copy.at<Vec3b>(i,j)[1] = col_copy.at<Vec3b>(i,j)[2] = map.at<uchar>(i,j);
		}
	}

	//Draw locations as blue coloured dots
	for(i=0;i<locations.size();i++){
		circle(col_copy,locations[i],3,Scalar(255,0,0),-1);
	}

	//Display coloured map
	// imshow("Locations",col_copy);
	// waitKey(0);

	col_copy.release();

}

void Coverage::GetCentroids(Mat& img,vector<Point>& sites, vector<Point>& centroids)
{
	//Obtain Centroids of partitions using blob method
	//Partitions will be formed by lines in the image

	int n_agents = sites.size(),i,r=img.rows,c=img.cols,k,l;

	//To remove adding duplicates to BFS queue
	vector< vector<bool> > visited(r,vector<bool>(c,false));

	for(i=0;i<n_agents;i++){

		//Begin BFS for centroids
		queue<Point> frontier;
		frontier.push(sites[i]);

		visited[sites[i].y][sites[i].x] = true;
		double avgx=0,avgy=0;
		int n_pix=0;

		while(!frontier.empty()){
			Point top = frontier.front();
			frontier.pop();
			n_pix++;
			avgx += top.x;
			avgy += top.y;

			for(k=top.y-1;k<=top.y+1;k++){
				for(l=top.x-1;l<=top.x+1;l++){
					if(IsValidPoint(l,k,r,c)==false||img.at<uchar>(k,l)==0||visited[k][l]==true)
						continue;
					Point nbr(l,k);
					visited[k][l] = true;
					frontier.push(nbr);
				}
			}
		}

		//Get avg values and centroid

		avgx/=n_pix;
		avgy/=n_pix;

		
		Point centroid(avgx,avgy);
		centroids[i] = centroid;

	}

}

void Coverage::GetStartingPoints(vector<Point>& sites)
{
	//Get initial non-blocked starting points for each agent, from the map
	sites.resize(n_agents);
	int i,j,r=map.rows,c=map.cols;

	//Equally divide 2d array (as 1d array) into as many segments as agents
	int len = r*c;
	int step = len/n_agents;
	int pos = step/2+step/4;

	//Assign agent of segment to nearest free position
	for(i=0;i<n_agents;i++){
		j = pos;
		while(map.at<uchar>(j/c,j%c)==0)
			j=(j+1)%len;
		sites[i] = Point(j%c,j/c);
		pos = (pos+step)%len;
	}

}

void Coverage::GetBestPositions(vector<Point>& positions)
{
	//Overall method that returns the best positions for the map
	vector<Point> sites;
	GetStartingPoints(sites);
	DisplayPoints(sites);

	int i,j;
	double sumdist = 1000000;

	vector<float> xValues(n_agents); //Voronoi object takes two separate arrays as args
	vector<float> yValues(n_agents);
	vector<Point> temp_sites(sites); //Copy the site points to temp variable
	vector<Point> temp_centres(n_agents);

	VoronoiDiagramGenerator vdg;

	Mat work_copy; //To display lines and use for centroids

	while(sumdist > 4*n_agents) //Threshold for convergence
	{
		work_copy = map.clone();

		for(i=0;i<n_agents;i++){
			xValues[i] = temp_sites[i].x;
			yValues[i] = temp_sites[i].y;
		}

		//Get Voronoi Diagram from object
		vdg.generateVoronoi(xValues,yValues,n_agents,0,map.cols-1,0,map.rows-1,3);
		vdg.resetIterator();

		float x1,x2,y1,y2; //To get co-ords of returned lines

		while(vdg.getNext(x1,y1,x2,y2)){
			//Extract truncated lines one by one 
			line(work_copy,Point(x1,y1),Point(x2,y2),Scalar(0),2);
		}

		GetCentroids(work_copy,temp_sites,temp_centres);

		sumdist = 0.0;
		//Set sites to centroids
		for(i=0;i<n_agents;i++){
			sumdist+= EuclideanDist(temp_sites[i],temp_centres[i]);
			temp_sites[i] = temp_centres[i];
		}
	}

	//Shift blocked centres to nearest free point
	for(i=0;i<n_agents;i++){
		if(IsBlocked(temp_centres[i]))
			temp_centres[i] = GetNearestFreeCentroid(temp_centres[i]);
	}

	//Final answer is the best estimate
	positions = temp_centres;
	for(i=0;i<positions.size();i++){
		circle(work_copy,positions[i],3,Scalar(125),-1);
	}	
	//Show final partitions
	imwrite("Partition.png",work_copy);
	// waitKey(0);

}


