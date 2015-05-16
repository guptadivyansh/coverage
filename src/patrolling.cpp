#include <iostream>
#include <stack>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "patrolling.h"
#include <cassert>
#include <cstdlib>
#include <patrolling.h>
#include "VoronoiDiagramGenerator.h"

using namespace std;
using namespace cv;

Patrolling::~Patrolling()
{
	map.release();
}

int Patrolling::GetInitCellAssignments(vector<vector<int> >& assignments, vector<int>& starting)
{
	//Obtains initial assignment of grid cells to 1 or more agents - through 'assignments'
	//Also gets index of north-west most cell for each agent (least index value) - through 'starting'
	//Return value is the count of 'doubtful' cells

	int i,count_doubt=0;
	int n_vertices = graph_obj.GetNVertices(); //Number of vertices in overlaid graph

	//Re-initialize in case parameters passed incorrectly
	assignments.resize(n_vertices,vector<int>());
	starting.resize(n_agents,n_vertices);


	for(int i=0;i<n_vertices;i++){

		//Get the minimum and 2nd minimum 
		//YOU GUYS SHOULD GENERALIZE THIS TO STORING ALL
		//AND THEN RETURNING AS DOUBTFUL ALL THOSE WHICH ARE LESS THAN 2R FROM MIN

		Point p = graph_obj.v_list[i].v_pt; //The graph builder object

		double min1 = map.rows+map.cols,min2 = map.rows+map.cols; //THIS SHOULD BE EXTENDED TO AN ARRAY
		int idx1=-1,idx2=-1; //SAME AS PREV COMMENT

		//In subsequent loop, instead of just getting min and 2nd min
		//you should get extend generally for all n_agents
		for(int j=0;j<n_agents;j++){

			double dist = EuclideanDist(p,centroids[j]);

			if(dist<min1){
				min1 = dist;
				idx1 = j;
			}
			else if(dist<min2){
				min2 = dist;
				idx2 = j;
			}
		}


		//This is where you check if multiple can be assigned to it
		//Change accordingly here too

		if(min2 - min1 > 2*radius){
			//Fully assigned to one cell
			if(i < starting[idx1])
				starting[idx1] = i;

			assignments[i].push_back(idx1);
		}
		else{
			//Assigned to multiple cells (here two, but change as needed)
			count_doubt++;
			assignments[i].push_back(idx1);
			assignments[i].push_back(idx2);
		}
	}

	return count_doubt;

}



double Patrolling::GetPathForIndex(int idx, bool cw, vector<vector<int> > assignments,
									int count_cells, int start_idx, vector<int>& best_index_path)
{
	//Given a set of assignments, the index of the agent, and the number of cells PURELY assigned to that agent
	//Find the path that visits all purely assigned cells - returned through 'best_index_path'
	//cw == true means go clockwise, else go anti-clockwise
	//Return value is the turnaround time value as discussed - number of cells + end-start as straight line

	double turnaround = -1.0; //Will be returned

	int n_vertices = graph_obj.GetNVertices(); //Number of vertices in overlaid graph
	int c_map = map.cols/(2*radius); //Number of vertices (columns) per row in grid-graph
	
	stack<int> path_stack; //Stack for storing vertices along path
	vector<int> nbr_inc(4); //Incremental values to get neighbour vertices, based on CW/ACW
	vector<bool> visited(n_vertices,false); //To avoid counting/adding visited cells
	path_stack.push(start_idx); //Begin with starting vertex


	if(cw==true){
		//Right, Down, Left, Up
		nbr_inc[0] = 1; nbr_inc[1] = c_map; nbr_inc[2] = -1; nbr_inc[3] = -c_map;
	}
	else{
		//Down, Right, Up, Left
		nbr_inc[0] = c_map; nbr_inc[1] = 1; nbr_inc[2] = -c_map; nbr_inc[3] = -1;
	}

	int count = 0; //For keeping a track of how many cells covered

	while(count < count_cells)
	{
		int top = path_stack.top(); //Top only accessed, NOT popped (as explained)
		best_index_path.push_back(top); //Added to path - may be repetitions when backtracking


		if(!visited[top]){
			//Only count each new cell once
			visited[top] = true;
			count++;
		}

		int i;
		for(i=0;i < 4;i++){
			//Get potential neighbour indices
			int nbr = top + nbr_inc[i];

			if(nbr < 0 || nbr >= n_vertices) //Should not be out-of-bounds
				continue;
			if(visited[nbr]==true ||
				assignments[nbr].size()>1 || 
				assignments[nbr][0] != idx) //Should not be visited, doubtful, or belong to another agent
				continue;
			
			//Below commented block is to avoid adding vertices in black pixels - NAIVE APPROACH
			//SEE IF YOU CAN REFINE

			/*Point p = graph_obj.v_list[nbr].v_pt;
			if(map.at<Vec3b>(p.y,p.x)[0] == 0) //Black pixel for vertex
				continue;*/

			//Add to stack ONLY the first neighbour (in order) that survives above checks
			path_stack.push(nbr);
			break;
		}

		if(i==4) //No valid neighbour added
			path_stack.pop(); //Backtrack

		if(path_stack.empty()) //THIS WILL HAPPEN IF SOME VERTEX IS UNREACHABLE
			return n_vertices*n_vertices; //Return an impossibly high turnaround time so it is never considered
	}


	turnaround = best_index_path.size()*1.0;

	//Currently turnaround time is number of cells
	int last_idx = best_index_path.back();

	//Now add to it the scaled distance between last and start
	turnaround += EuclideanDist(graph_obj.v_list[start_idx].v_pt,graph_obj.v_list[last_idx].v_pt);

	return turnaround;
}

vector<vector<Point> > Patrolling::GetBestPatrol()
{
	graph_obj.GenerateRoadMap(map.rows,map.cols,2*radius); //Roadmap basically means that whole graph - just a term
	int n_vertices = graph_obj.GetNVertices(); //Number of vertices in graph

	//The following data members will be used for various computations
	//They will be initialized before the main loop
	vector<vector<int> > assignments;
	vector<int> starting;
	vector<double> turnaround_times(n_agents,0.0);
	vector<int> count_cells(n_agents,0);

	//Get doubtful cells and initial assignments and start cells
	int count_doubt = GetInitCellAssignments(assignments,starting);


	//Count the non-doubtful cells for each agent
	for(int i=0;i<n_vertices;i++){

		if(assignments[i].size()==1){
			count_cells[assignments[i][0]]++;
		}
	}

	//Get initial best turnaround times for each agent
	for(int i=0; i < n_agents; i++){

		vector<int> index_path1;
		vector<int> index_path2;

		double min1 = GetPathForIndex(i,true,assignments,count_cells[i],starting[i],index_path1);
		double min2 = GetPathForIndex(i,false,assignments,count_cells[i],starting[i],index_path2);

		turnaround_times[i] = (min1<min2)?min1:min2; //Get lesser of CW and ACW
	}


	//Now run main loop to assign each doubtful cell one-by-one
	while(count_doubt > 0){

		//Minimize increase in turnaround time over all doubtful cells and all assignments
		double min_inc_all = n_vertices*n_vertices;
		int min_idx_all=-1;
		int min_cell = -1;

		//Iterate over each doubtful cell
		for(int i=0;i<n_vertices;i++){

			if(assignments[i].size()==1) //Not doubtful cell
				continue;


			//Minimize increase in turnaround time over all assignments of that doubtful cell
			double min_inc_idx = n_vertices*n_vertices;
			int min_idx = -1;

			//Consider all possible assignments of doubtful cell
			int poss = assignments[i].size();

			for(int j=0; j < poss; j++){

				vector<vector<int> > assignments_copy(assignments); //Make a copy of assignments
				int idx = assignments[i][j];

				//Assign the doubtful cell to only the current possibility
				assignments_copy[i].clear();
				assignments_copy[i].push_back(idx);

				vector<int> index_path1;
				vector<int> index_path2;

				//Get minimum of CW/ACW for that possible assignment of the cell
				double min1 = GetPathForIndex(idx,true,assignments_copy,count_cells[idx]+1,starting[idx],index_path1);
				double min2 = GetPathForIndex(idx,false,assignments_copy,count_cells[idx]+1,starting[idx],index_path2);
				double min_for_idx = (min1<min2)?min1:min2;

				double inc = min_for_idx - turnaround_times[idx];

				//Get smallest increment for that cell
				if(inc < min_inc_idx){
					min_inc_idx = inc;
					min_idx = idx;
				}
			}

			//Get smallest increment overall
			if(min_inc_idx < min_inc_all){
				min_inc_all = min_inc_idx;
				min_idx_all = min_idx;
				min_cell = i;

			}

		}

		//Cell min_cell has been assigned to agent min_idx_all
		//Change assignments to reflect that
		assignments[min_cell].clear();
		assignments[min_cell].push_back(min_idx_all);

		//Change start if smaller comes
		if(min_cell < starting[min_idx_all])
			starting[min_idx_all] = min_cell;

		//Change cell_counts, turnaround times accordingly
		count_cells[min_idx_all]++;
		turnaround_times[min_idx_all] += min_inc_all;

		//One less doubtful cell to assign
		count_doubt--;

	}

	
	//Now get final best paths for each agent
	vector<vector<int> > best_paths(n_agents,vector<int>());

	for(int i=0;i<n_agents;i++){
		vector<int> index_path1;
		vector<int> index_path2;

		double min1 = GetPathForIndex(i,true,assignments,count_cells[i],starting[i],index_path1);
		double min2 = GetPathForIndex(i,false,assignments,count_cells[i],starting[i],index_path2);

		if(min1<min2)
			best_paths[i] = index_path1;
		else
			best_paths[i] = index_path2;
	}

	vector<vector<Point> > point_path=vector<vector<Point> >(n_agents,vector<Point>());

	for(int i=0; i < n_agents;i++){
		for(vector<int>::iterator it=best_paths[i].begin();it!=best_paths[i].end();it++){
			Point p = graph_obj.v_list[*it].v_pt;
			point_path[i].push_back(p);
		}
	}
	return point_path;
	//(not Shushman) Instead of printing, return the paths so that they can be published

	//This part was for displaying video of patrolling
	
	/*namedWindow("Patrolling",CV_WINDOW_NORMAL);
	vector<int> path_indices(n_agents,0);

	while(true){

		Mat copy = map.clone();

		for(int i=0; i < n_agents;i++){

			int idx = path_indices[i];
			Point p = graph_obj.v_list[best_paths[i][idx]].v_pt;
			circle(copy,p,radius,Scalar(245,0,0),-1);

			path_indices[i] = (idx+1)%best_paths[i].size();

		}

		imshow("Patrolling",copy);
		char ch = waitKey(50);
		if(ch==27)
			break;
	}*/


	//No return value but you can return paths if you wish
	//Or send a reference to vector of paths as a param

}


