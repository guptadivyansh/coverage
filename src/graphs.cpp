#include "graphs.h"
#include <vector>

using namespace std;
using namespace cv;


void Graph::AddVertex(int id, Point& pt)
{
    //Adds to the list of vertices
    Vertex v(id,pt);
    v_list.push_back(v);
    vector<int> f;
    adj_list.push_back(f);
}

void Graph::AddEdge(int u, int v)
{
    //I am not storing any explicit info related to edges
    //Like edge-list or whatever
    //But you might find it useful to add stuff for that

    adj_list[u].push_back(v);
    adj_list[v].push_back(u);
}

void Graph::GenerateRoadMap(int rows, int cols, int step)
{
    //This method generates the graph that is overlaid on the image
    //Number of rows and columns in roadmap
	int r_map = rows/step, c_map = cols/step;

	int i,j,id=0;

    //Vertices are generated in row-major manner
	for(i = 0; i < r_map; i++){

        //Y-coordinate of current row
		int y = i*step + step/2;

        //Vertices of each column for that row added
        for(j = 0; j< c_map; j++){

        	int x = j*step + step/2;

        	Point pt(x,y);
        	AddVertex(id++,pt);
        }
    }

    n_vertices = adj_list.size();

    id=0; //Now this counts the number of edges

    //You might find yourself not needing explicit edges to be added
    //by the initial method I proposed here
    //Since I am directly computing neighbour IDs from each node
    //But you may wish to extend it later so I have kept some framework

    for(i = 0; i < r_map-1; i++){
        for(j = 0; j < c_map-1; j++){

            int cur_v_id = i*c_map + j;

            //See how the vertex IDs are related to neighbours
            AddEdge(cur_v_id,cur_v_id+1);
            AddEdge(cur_v_id,cur_v_id+c_map);
            AddEdge(cur_v_id,cur_v_id+c_map+1);
            AddEdge(cur_v_id+1,cur_v_id+c_map);

            id+=4;
        }
    }

    for(i=0;i<r_map-1;i++){
        int cur_v_id = i*c_map+c_map-1;
        AddEdge(cur_v_id,cur_v_id+c_map);
        id++;
    }

    for(j=0;j<c_map-1;j++){
        int cur_v_id = (r_map-1)*c_map + j;
        AddEdge(cur_v_id,cur_v_id+1);
        id++;
    }

    n_edges = id;
}