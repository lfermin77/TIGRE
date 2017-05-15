#include "graph.hpp"



////Graph as adjacency list nested class

node_info::node_info(){
	distance_from_origin= std::numeric_limits<float>::infinity();
	label=-1;
	region_label=-1;
	sub_region=-1;
}
/////////////////////////////

edge_info::edge_info(){
	distance = -1;
	label=-1;
}


///////////////////
	
Node::Node(){
	predecesor=NULL;
}


void Node::print_node_label_and_pos(){
	std::cout << "Node "<< info.label<< ", region "<< info.region_label << std::endl;
	for(int i=0; i < connected.size();i++){
		std::cout << "connected to "<< (connected[i].to)->info.label<<std::endl;
		std::cout << "connected in ("<< (connected[i].linker)->from->info.label<<","<< (connected[i].linker)->to->info.label << ")"<<std::endl;
	}
}







////////////////////////////////////////////////////////
///////////////////////////////////////////////////


	

UtilityGraph::~UtilityGraph(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++)
	    delete *it; 
	Nodes.clear();

	for(Edge_iter it = Edges.begin(); it != Edges.end(); it++)
	    delete *it;
	Edges.clear();

}



void UtilityGraph::print_nodes(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++){
//		(*it)->print_node_label_and_pos();
		std::cout << "Labels "<<(*it)->info.label<< " at distance " <<  (*it)->info.distance_from_origin << ", region "<< (*it)->info.region_label <<  std::endl;
	}
}



Node_iter UtilityGraph::find_point_in_node(std::complex<double> query_position){
	Node_iter first = Nodes.begin();
	Node_iter last = Nodes.end();

	while (first!=last) {
		if ((*first)->info.position == query_position) return first;
		++first;
	}
	return last;
}

int UtilityGraph::update_distances(	std::complex<double> current_position ){
	std::list <Node*> Unvisited_Nodes = Nodes; //List to work

	//// Find source node
	Node* Minimal_Node;
	Node_iter Source_iter = Unvisited_Nodes.end();
	for(Node_iter it = Unvisited_Nodes.begin(); it != Unvisited_Nodes.end(); it++){
		if((*it)->info.position ==  current_position){
			Minimal_Node = (*it);
			Source_iter = it;
		}
	}	
	Minimal_Node->predecesor = NULL;
	Minimal_Node->info.distance_from_origin = 0;
	
//	std::cout << "Minimun distace found, is  " << Minimal_Node->info.label << std::endl;

	while(Unvisited_Nodes.size() > 0){
		Node_iter eliminate_iter = Unvisited_Nodes.begin();
		float min_distance = std::numeric_limits<float>::infinity();

		for(Node_iter it = Unvisited_Nodes.begin(); it != Unvisited_Nodes.end(); it++){
			if( ((*it)->info.distance_from_origin ) < min_distance ){
				eliminate_iter = it;
				min_distance = (*it)->info.distance_from_origin;
			}
		}
		Minimal_Node = (*eliminate_iter);

		Unvisited_Nodes.erase(eliminate_iter);
		
		for(int i=0; i < Minimal_Node->connected.size();i++){
			float new_distance = Minimal_Node->info.distance_from_origin  +  Minimal_Node->connected[i].linker->info.distance;

			if(new_distance < Minimal_Node->connected[i].to->info.distance_from_origin  ){
				Minimal_Node->connected[i].to->info.distance_from_origin = new_distance;
				Minimal_Node->connected[i].to->predecesor = Minimal_Node;
			}
		}
	}
	return -1;
}


std::list <Edge*> UtilityGraph::find_edges_between_regions(){
	std::list <Edge*> connecting_edges;
	std::cout << "Connecting Edges "<< std::endl;
		
	for(Edge_iter it = Edges.begin(); it != Edges.end(); it++){
		int region_from = (*it)->from->info.region_label;
		int region_to   = (*it)->to->info.region_label;

			
		if( region_from != region_to ){
			connecting_edges.push_back((*it) );
//			std::cout <<"Edge "<< (*it)->info.label << " connect the region " << region_from  <<" with "<< region_to << std::endl;
		}
		
		
	}
	return connecting_edges;
}


void UtilityGraph::evaluate_regions_connectivity(){
	std::map < int, std::list <Node*> > labels_to_nodes;
	
	std::vector < std::vector < std::list <Node*> > >   Regions_in_Graph;
	std::vector < std::list <Node*> >    Sub_Regions_in_Regions;
	Regions_in_Graph.clear();	
	
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++){
		int region_label = (*it)->info.region_label;
		labels_to_nodes[region_label].push_back(*it);
	}
		
	for(std::map < int, std::list <Node*> >::iterator map_it = labels_to_nodes.begin(); map_it != labels_to_nodes.end() ; map_it++){
		int sub_region=0;
		std::list <Node*> Remaining_Nodes, List_of_Nodes = map_it->second, Nodes_in_sub_Region;
		
		Sub_Regions_in_Regions.clear();
		
		while( List_of_Nodes.size() > 0){// Breadth first
			evaluate_list_connectivity(List_of_Nodes, sub_region);
			Remaining_Nodes.clear();
			Nodes_in_sub_Region.clear();

			for(Node_iter it = map_it->second.begin(); it != map_it->second.end(); it++){		
				if( (*it)->info.sub_region == -1 ){
					Remaining_Nodes.push_back(*it);
				}
				if( (*it)->info.sub_region == sub_region ){
					Nodes_in_sub_Region.push_back(*it);
				}
			}
			List_of_Nodes = Remaining_Nodes;
			sub_region++;
			Sub_Regions_in_Regions.push_back(Nodes_in_sub_Region);
		}

		Regions_in_Graph.push_back(Sub_Regions_in_Regions);
	}

	std::cout << "The graph is decomposed in " << Regions_in_Graph.size()<<" Regions" <<std::endl;
		std::map < int, std::list <Node*> >::iterator map_it = labels_to_nodes.begin();
	for(int i=0; i< Regions_in_Graph.size(); i++){
		
		std::cout << " Region: " << map_it->first <<" with "<<Regions_in_Graph[i].size()<<" subregions" <<std::endl;
		std::vector < std::list <Node*> > Subregions_Inside = Regions_in_Graph[i];
		
		for(int j=0; j< Subregions_Inside.size(); j++){
			std::cout << "   Sub_Region: " << j <<" with "<< Subregions_Inside[j].size()<<" nodes"<<std::endl;
		}
		map_it++;
	}

}

void UtilityGraph::evaluate_list_connectivity(std::list <Node*> list_in, int name){

	list_in.front()->info.sub_region = name;
	
	int current_region_label = list_in.front()->info.region_label;

	std::queue<Node*> Q;
	Q.push( list_in.front() );

	
	while (!Q.empty()){
		Node* current = Q.front();		Q.pop();

		for (int i=0;i< current->connected.size();i++ ){
			Node* destiny =current->connected[i].to;

			if(destiny->info.region_label == current_region_label){
				if(destiny->info.sub_region != name ){	
					destiny->info.sub_region = name;
					Q.push(destiny);
				}
			}
		}
	} 
}


void UtilityGraph::Closest_subregions(std::list <Node*> node_pair, int region){
	std::map < int , std::vector<Node*> > subregion_nodes;
	
	for(std::list <Node*>::iterator node_it = Nodes.begin(); node_it != Nodes.end(); node_it++  ){
		if ( (*node_it)->info.region_label == region){
			subregion_nodes[ (*node_it)->info.sub_region ].push_back( *node_it  ); 
		}		
	}
	/////////
	for(int i=1; i < subregion_nodes.size() ; i++){
		std::cout << "Subregion "<< i << " connected to " << std::endl;
		for(int j=0; j < i; j++){
			std::cout << "  "<< j ;//<< std::endl;
			closest_node_subregions(subregion_nodes[i] , subregion_nodes[j] );
		}
	}
	/////////
}


std::pair < Node* , Node* > UtilityGraph::closest_node_subregions(std::vector<Node*> path_1, std::vector<Node*> path_2){
	
	std::pair < Node* , Node* > minimum_pair;
	float minimun_distance = std::numeric_limits<float>::infinity();

	//Slow approach
	for (std::vector<Node*>::iterator path_1_iter =path_1.begin(); path_1_iter != path_1.end(); path_1_iter ++){
		for (std::vector<Node*>::iterator path_2_iter =path_2.begin(); path_2_iter != path_2.end(); path_2_iter ++){
			float distance_between_nodes = abs( (*path_1_iter)->info.position - (*path_2_iter)->info.position );
			if (distance_between_nodes < minimun_distance){
				minimum_pair.first  = *path_1_iter;
				minimum_pair.second = *path_2_iter;
				minimun_distance = distance_between_nodes;
			}
			
		}
	}
		std::cout << " at "<< minimun_distance << " m" << std::endl;
	
	return minimum_pair;
}	

int UtilityGraph::build_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers){
	
	UtilityGraph UGraph = *this;
	int number_of_edges = edge_markers.size()/2;				
	int labeler=0;
	
	Node* from_Node_ptr; 
	Node* to_Node_ptr;   
			
	for (int i=0; i < number_of_edges;i++){
	
//		cout << "Insert nodes"<< endl;
		// FROM
		std::complex<double> FROM_position(edge_markers[2*i].x, edge_markers[2*i].y);
//				std::cout << "label " << 100*edge_markers[2*i].z << std::endl;
					
		Node_iter FROM_node_iter = UGraph.find_point_in_node(FROM_position);		
		if(FROM_node_iter == UGraph.Nodes.end() ){//couldn't find, insert new node
			labeler++;
			from_Node_ptr =new Node;
			from_Node_ptr->info.position = FROM_position;
			from_Node_ptr->info.label = 100*edge_markers[2*i].z;//labeler;
			UGraph.Nodes.push_back(from_Node_ptr);	
		}			
		else from_Node_ptr = *FROM_node_iter;
		
		// TO
		std::complex<double> TO_position(edge_markers[2*i+1].x, edge_markers[2*i+1].y);			
		Node_iter TO_node_iter = UGraph.find_point_in_node(TO_position);			
		if(TO_node_iter == UGraph.Nodes.end() ){//couldn't find, insert new node
			labeler++;
			to_Node_ptr = new Node;
			to_Node_ptr->info.position = TO_position;
			to_Node_ptr->info.label = 100*edge_markers[2*i+1].z;//labeler;
			UGraph.Nodes.push_back(to_Node_ptr);				
		}			
		else to_Node_ptr = *TO_node_iter;


//		cout << "Insert Edges"<<endl;
		Edge* current_edge = new Edge;
		current_edge->info.distance = abs(FROM_position - TO_position);
		current_edge->info.label = i;
		
		current_edge->from = from_Node_ptr;
		current_edge->to   = to_Node_ptr;  
		
		UGraph.Edges.push_back(current_edge);
	
//		cout << "Insert Linked Information"<<endl;
		Connections connecting_from;
		connecting_from.linker = current_edge;			
		connecting_from.to = to_Node_ptr;
		from_Node_ptr->connected.push_back(connecting_from);
		
		Connections connecting_to;
		connecting_to.linker = current_edge;			
		connecting_to.to = from_Node_ptr;
		to_Node_ptr->connected.push_back(connecting_to);
		//Edges
		current_edge->from = from_Node_ptr;
		UGraph.Edges.back()->to = to_Node_ptr;
	}
	std::cout << "Found " << labeler  <<" nodes"<< std::endl;

	return 1;
}


int UtilityGraph::build_SLAM_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers){
	int a=1;
}


cv::Mat UtilityGraph::graph2image(nav_msgs::MapMetaData info, cv::Mat  Tag_image ){
	
	UtilityGraph GraphSLAM =*this;
	
	cv::Mat  Node_image  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	cv::Mat  image_test  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	
	std::complex<double> origin(info.origin.position.x, info.origin.position.y);	

	for(Node_iter it = GraphSLAM.Nodes.begin(); it != GraphSLAM.Nodes.end(); it++ ){
		std::complex<double> current_node_position = (*it)->info.position;

		current_node_position = (current_node_position - origin)/(double)info.resolution;
		int x = round(current_node_position.real() );
		int y = round(current_node_position.imag() );

		int current_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y));

		Node_image.at<uchar>(cv::Point(x, info.height - y)) = current_tag;
		
		(*it)->info.region_label = current_tag -1;
	}

	if(false){
		return  Node_image;
	}
	else{
		Node_image=Node_image>0;
		cv::dilate(Node_image, Node_image, cv::Mat(), cv::Point(-1,-1), 3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			
		Tag_image.copyTo(image_test , ~Node_image);
		return  image_test;
//				return  Node_image;
	}
}







cv::Mat UtilityGraph::build_region_graph(cv::Mat  Tag_image, cv::Mat  original_image){
	
	int window_size=1;
	
	cv::Mat  Frontier_image  = cv::Mat::zeros(original_image.size(), CV_8UC1);
	
	
	edge_points_mapper mapping_set_to_point_array, mapping_frontier_to_point_array;
	region_points_mapper mapping_region_to_point_array;
	
	for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
		for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
		
			/////////////////////
			cv::Point window_center(i,j);
			int center_tag = Tag_image.at<uchar>(window_center);
			
			std::set<int>  connections_in_region, frontier_connections;
			//check neigbourhood for frontiers and connection
			for(int x=-window_size; x <= window_size; x++){
				for(int y=-window_size; y <= window_size; y++){
					cv::Point delta(x,y); 							
					cv::Point current_point = window_center + delta;
					int tag = Tag_image.at<uchar>(current_point);
					int frontier = original_image.at<uchar>(current_point);
					
					if (tag>0){
						connections_in_region.insert( tag -1 );
						frontier_connections.insert( tag -1 );
					}
					if ( frontier==255 &&  tag==0) frontier_connections.insert( -1 );

				}
			}
			//////////////////////////////
//*
			if (connections_in_region.size()==2 || (frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ) ){
				mapping_region_to_point_array[center_tag-1].push_back(window_center);
			}
//*/			
			//////////////////
			if(connections_in_region.size()==2){					
				mapping_set_to_point_array[connections_in_region].push_back(window_center);
				
			}
			if(frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ){					
				mapping_frontier_to_point_array[frontier_connections].push_back(window_center);
	//					Frontier_image.at<uchar>( window_center ) = 255;
			}
		}
	}
	//////////////	
	
	
	
	for (region_points_mapper::iterator it2 = mapping_region_to_point_array.begin(); it2 != mapping_region_to_point_array.end(); it2 ++){
		
		Region_Node *InsideNode;
		InsideNode = new Region_Node;
		InsideNode->contour = (*it2).second;
		
		Region_Nodes.push_back(InsideNode);
	}
	
	
//*
	for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){

/*
		Region_Edge *InsideEdge;
		InsideEdge = new Region_Edge;
//*/
		std::cout << "Connections  are: ";
		std::set<int> current_connections_set = it2->first ;
		for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
			std::cout <<" " << *it;
		}
		
		std::vector<cv::Point> current_points = it2->second;
		cv::Point average_point(0,0);

		for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
			average_point += *it;
		}		
		std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ")";
		
		std::cout << std::endl;
	}
	//*/
	
/*
	std::cout << "Frontiers size is: "<<  mapping_frontier_to_point_array.size() << std::endl;
	for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){
		
		if(it2->first.size() <3){
			std::cout << "Frontiers  are: ";
			std::set<int> current_connections_set = it2->first ;
			for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
				std::cout <<" " << *it;
			}
			
			std::vector<cv::Point> current_points = it2->second;
			cv::Point average_point(0,0);	
			for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
				average_point += *it;
				Frontier_image.at<uchar>( *it ) = 255;
			}		
			std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ") size: "<< current_points.size();
			
			std::cout << std::endl;

		}

	}
	return Frontier_image;
//			return original_image;
	//*/



		

}










///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////




int RegionGraph::build_Region_Graph(std::vector<geometry_msgs::Point> edge_markers, nav_msgs::MapMetaData info, cv::Mat  Tag_image, cv::Mat  original_image){
	
	int number_of_edges = edge_markers.size()/2;				
	
	Node* from_Node_ptr; 
	Node* TO_Node_ptr;   
		
	std::complex<double> origin(info.origin.position.x, info.origin.position.y);	
	
	
	for (int i=0; i < number_of_edges;i++){
		std::complex<double> FROM_position( edge_markers[2*i].x , edge_markers[2*i].y  );
		std::complex<double> TO_position  (edge_markers[2*i+1].x, edge_markers[2*i+1].y);

		int FROM_label = edge_markers[2*i].z*100;
		int TO_label   = edge_markers[2*i+1].z*100;


		NodeMapper::iterator FROM_iter = Nodes_Map.find (FROM_label);
		NodeMapper::iterator TO_iter = Nodes_Map.find (TO_label);


//		std::cout << "FROM NODE"<< std::endl;
		if(FROM_iter == Nodes_Map.end() ){ //not found
			from_Node_ptr = new Node;
			from_Node_ptr->info.position = FROM_position;
			from_Node_ptr->info.label = FROM_label;
			
			////Find Region label
			FROM_position = (FROM_position - origin)/(double)info.resolution;
			int x = round(FROM_position.real() );
			int y = round(FROM_position.imag() );
			
			int region_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y)) - 1;
			
			from_Node_ptr->info.region_label = region_tag;
			
			
			// Insert Region Node
			{
				RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
				if(Region_iter == Region_Nodes_Map.end() ){// not found
					Region_Node* current_region = new Region_Node;
					current_region->nodes_inside.push_back(from_Node_ptr);
					Region_Nodes_Map[region_tag] = current_region;				
				}
				else{
					Region_Nodes_Map[region_tag]->nodes_inside.push_back(from_Node_ptr);
				}
			//////////
			}
			Nodes_Map[FROM_label] = from_Node_ptr;	//Inser Region Map
		}
		else {
			from_Node_ptr = (*FROM_iter).second;
		}



//		std::cout << "TO NODE"<< std::endl;
		if(TO_iter == Nodes_Map.end() ){ //not found
			TO_Node_ptr = new Node;
			TO_Node_ptr->info.position = TO_position;
			TO_Node_ptr->info.label = TO_label;
			
			////Find Region label
			TO_position = (TO_position - origin)/(double)info.resolution;
			int x = round(TO_position.real() );
			int y = round(TO_position.imag() );
			
			int region_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y)) - 1;
			
			TO_Node_ptr->info.region_label = region_tag;
			
			
			// Insert Node to Regions
			{
				RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
				if(Region_iter == Region_Nodes_Map.end() ){// not found
					Region_Node* current_region = new Region_Node;
					current_region->nodes_inside.push_back(TO_Node_ptr);
					Region_Nodes_Map[region_tag] = current_region;				
				}
				else{
					Region_Nodes_Map[region_tag]->nodes_inside.push_back(TO_Node_ptr);
				}
			//////////
			}
			Nodes_Map[TO_label] = TO_Node_ptr;	//Inser Region Map
		}
		else {
			TO_Node_ptr = (*TO_iter).second;
		}
		
		
//		std::cout << "Insert EDGES"<< std::endl;
		Edge* current_edge = new Edge;
		current_edge->info.distance = abs(FROM_position - TO_position);
		current_edge->info.label = i;
		
		current_edge->from = from_Node_ptr;
		current_edge->to   = TO_Node_ptr;  

		Edges_Map[i] = current_edge;


//		std::cout << "Insert Linked Information"<< std::endl;
		Connections connecting_from;
		connecting_from.linker = current_edge;			
		connecting_from.to = TO_Node_ptr;
		from_Node_ptr->connected.push_back(connecting_from);
		
		Connections connecting_to;
		connecting_to.linker = current_edge;			
		connecting_to.to = from_Node_ptr;
		TO_Node_ptr->connected.push_back(connecting_to);
		//Edges
		current_edge->from = from_Node_ptr;
		current_edge->to = TO_Node_ptr;
	}

	std::cout << "Everything inserted "<< std::endl;


	
	
	///////
	return 1;
}


RegionGraph::~RegionGraph(){
	

	
	for(NodeMapper::iterator it = Nodes_Map.begin(); it != Nodes_Map.end(); it++){
		Node* ptr = (*it).second;
	    delete  ptr;
	}
	Nodes_Map.clear();

	for(EdgeMapper::iterator it = Edges_Map.begin(); it != Edges_Map.end(); it++){
		Edge* ptr = (*it).second;
	    delete  ptr;
	}
	Edges_Map.clear();

	for(RegionNodeMapper::iterator it = Region_Nodes_Map.begin(); it != Region_Nodes_Map.end(); it++){
		Region_Node* ptr = (*it).second;
	    delete  ptr;
	}
	Region_Nodes_Map.clear();
	
	for(RegionEdgeMapper::iterator it = Region_Edges_Map.begin(); it != Region_Edges_Map.end(); it++){
		Region_Edge* ptr = (*it).second;
	    delete  ptr;
	}
	Region_Edges_Map.clear();


}










void RegionGraph::print_Region_Atributes(){


	std::cout << "Number of Nodes "<< Nodes_Map.size() << std::endl;
	std::cout << "Number of Edges "<< Edges_Map.size() << std::endl;
	std::cout << "Number of Regions "<< Region_Nodes_Map.size() << std::endl << std::endl;

	std::cout << "Number of Nodes per Region "<< std::endl;
	for (RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.begin(); Region_iter != Region_Nodes_Map.end(); Region_iter++){
		std::cout << "   Region "<< (*Region_iter).first  <<": Number of Nodes: "<< (*Region_iter).second->nodes_inside.size() << std::endl;
	}



}





/*		
		cv::Mat find_contour_connectivity_and_frontier(cv::Mat  Tag_image, cv::Mat  original_image){
			
			UtilityGraph Region_Graph;
			int window_size=1;
			
			cv::Mat  Frontier_image  = cv::Mat::zeros(original_image.size(), CV_8UC1);
			
			edge_points_mapper mapping_set_to_point_array, mapping_frontier_to_point_array;
			for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
				for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
				
					/////////////////////
					cv::Point window_center(i,j);
					
					std::set<int>  connections_in_region, frontier_connections;
					for(int x=-window_size; x <= window_size; x++){
						for(int y=-window_size; y <= window_size; y++){
							cv::Point delta(x,y); 							
							cv::Point current_point = window_center + delta;
							int tag = Tag_image.at<uchar>(current_point);
							int frontier = original_image.at<uchar>(current_point);
							
							if (tag>0){
								connections_in_region.insert( tag -1 );
								frontier_connections.insert( tag -1 );
							}
							if ( frontier==255 &&  tag==0) frontier_connections.insert( -1 );

						}
					}
					//////////////////
				if(connections_in_region.size()==2){					
					mapping_set_to_point_array[connections_in_region].push_back(window_center);
				}
				if(frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ){					
					mapping_frontier_to_point_array[frontier_connections].push_back(window_center);
//					Frontier_image.at<uchar>( window_center ) = 255;
				}
				}
			}
			//////////////	
			
			
//*
			for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){

				std::cout << "Connections  are: ";
				std::set<int> current_connections_set = it2->first ;
				for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
					std::cout <<" " << *it;
				}
				
				std::vector<cv::Point> current_points = it2->second;
				cv::Point average_point(0,0);

				for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
					average_point += *it;
				}		
				std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ")";
				
				std::cout << std::endl;
			}
			//*
			
//*
			std::cout << "Frontiers size is: "<<  mapping_frontier_to_point_array.size() << std::endl;
			for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){
				
				if(it2->first.size() <3){
					std::cout << "Frontiers  are: ";
					std::set<int> current_connections_set = it2->first ;
					for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
						std::cout <<" " << *it;
					}
					
					std::vector<cv::Point> current_points = it2->second;
					cv::Point average_point(0,0);	
					for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
						average_point += *it;
						Frontier_image.at<uchar>( *it ) = 255;
					}		
					std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ") size: "<< current_points.size();
					
					std::cout << std::endl;

				}

			}
			return Frontier_image;
//			return original_image;
			//*



				

		}

*/










