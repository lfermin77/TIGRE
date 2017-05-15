#include <limits>
#include <queue>
#include <list>
#include <map>
#include <complex>
#include <iostream>
#include <unordered_map>


#include <opencv2/imgproc/imgproc.hpp>
#include "visualization_msgs/Marker.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"



////Graph as adjacency list nested class
class node_info{
	public:
	int label;
	int region_label;
	int sub_region;
	float distance_from_origin;
	std::complex<double> position;
	
	node_info();
};

//////////////////////////
class edge_info{
	public:
	float distance;
	int label;
	
	edge_info();
};
/////////////////////
class Node;

class Edge{
	public:
	edge_info info;
	Node*  from;
	Node*  to;
};

///////////////////
struct Connections{
	Edge* linker;
	Node* to;
};

///////////////////////////
class Node{
	public:

	node_info info;	
	Node* predecesor;	
	std::vector<Connections> connected;
	
	Node();	
	void print_node_label_and_pos();
};


/////////////////////////////////////////////////
/////REGION GRAPH

class Region_Node; //forward declaration

//////
class Region_Sub_Edge{
	public:
	std::vector<cv::Point> frontier;
	std::vector <Edge*> Edges_in; //Refered to First Region
	std::vector <Edge*> Edges_out;
	
};
//////


class Region_Edge{
	public:
	std::vector <Edge*> Edges_in_border;
	std::vector<cv::Point> frontier;
	std::vector < std::vector<cv::Point> > segmented_frontier;
	float shortest_distance;
	std::set<int> Nodes_ids;
	
	Region_Node*  First_Region;
	Region_Node*  Second_Region;
	
	std::vector<Region_Sub_Edge> Sub_Edges;
};
///////////////////////////
class Region_Node{	
	public:
	int id;
	Node* Node_Center;
	std::list <Node*> nodes_inside;
	std::list < std::list <Node*> > sub_graphs;
	float distance_from_origin;
	Region_Node* predecesor;	
	std::vector<cv::Point> contour;
	std::vector<Region_Edge*> connected;
	
	Edge* Entrance_Edge;
	std::set< Edge*> marker_in;
	std::set< Edge*> marker_out;
};
////////////////////////////////////////////////////












////////////////////////////////////////////////////

typedef std::list<Node*>::iterator Node_iter;
typedef std::list<Edge*>::iterator Edge_iter;

///////////////////////////////////////
///////////////////////////////////////

typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;
typedef std::map < int , std::vector<cv::Point>   > region_points_mapper;

typedef	std::unordered_map <int,Node*> NodeMapper;
typedef	std::unordered_map <int,Edge*> EdgeMapper;
typedef	std::unordered_map <int,Region_Node*> RegionNodeMapper;
typedef	std::map < std::set<int> ,Region_Edge*> RegionEdgeMapper;
	

//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

class RegionGraph{
	public:
	
	~RegionGraph();
	int build_Region_Graph(std::vector<geometry_msgs::Point> edge_markers, nav_msgs::MapMetaData info, cv::Mat  Tag_image, cv::Mat  original_image);
	friend std::ostream& operator<<(std::ostream& os, RegionGraph& Graph);
	std::vector<std::complex<double> > collect_frontiers();
	std::vector<std::complex<double> > collect_all_frontiers();
	cv::Mat segment_current_frontier ( cv::Mat  Tag_image);

	int Tremaux_data( geometry_msgs::PoseStamped& pose_msg );
	int connect_inside_region( geometry_msgs::PoseStamped& pose_msg );
	int connect_inside_region_greedy( geometry_msgs::PoseStamped& pose_msg );

	int number_loop_closures();
	std::complex<double> Last_node_position(){return Nodes_Map[current_node_id]->info.position;};
	std::vector<float> extract_error_per_node(std::vector<geometry_msgs::Point>  gt_nodes, float & average);
	
	
	
	protected:
	void extract_subgraph();
	void evaluate_list_connectivity(std::list <Node*> list_in, int name);
	void build_region_graph(cv::Mat  Tag_image, cv::Mat  original_image);
	void find_edges_between_regions();
	void find_center_of_regions();
//	std::vector< std::vector < cv::Point > > segment_frontier (int region_id);
//	void segment_frontier (int region_id, cv::Mat  Tag_image);

	void segment_every_edge (int region_id, cv::Mat  Tag_image);
	cv::Mat segment_edge (std::set<int> edge_region_index, cv::Mat  Tag_image);
	geometry_msgs::PoseStamped extract_exploration_goal( std::vector<int> input_regions);
	geometry_msgs::PoseStamped choose_closer_frontier(std::vector<int> region);
	int check_map( geometry_msgs::PoseStamped& pose_msg );
	
	
	int current_node_id;
	nav_msgs::MapMetaData image_info;
	
	std::unordered_map <int,Node*> Nodes_Map;
	std::unordered_map <int,Edge*> Edges_Map;
	
	std::unordered_map <int,Region_Node*> Region_Nodes_Map;
	std::map < std::set<int> ,Region_Edge*> Region_Edges_Map;
	
		
};

