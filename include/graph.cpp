#include "graph.hpp"



geometry_msgs::PoseStamped construct_msg(std::complex<double> position, double angle){
	geometry_msgs::PoseStamped pose_msg;
	
	pose_msg.pose.position.x = position.real();
	pose_msg.pose.position.y = position.imag();
	pose_msg.pose.position.z = 0;

	pose_msg.pose.orientation.x = 0;
	pose_msg.pose.orientation.y = 0;
	pose_msg.pose.orientation.z = sin(angle/2);
	pose_msg.pose.orientation.w = cos(angle/2);
	
	return pose_msg;
}




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





///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////


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

///////////////////////
std::ostream& operator<<(std::ostream& os, RegionGraph& Graph){
	os << "\n"<< "\n";
	
	os << "Number of Nodes "<< Graph.Nodes_Map.size() << "\n";
	os << "Number of Edges "<< Graph.Edges_Map.size() << "\n";
	os << "Number of Regions "<< Graph.Region_Nodes_Map.size() << "\n" << "\n";

//	os << "Number of Nodes per Region "<< "\n";
	for (RegionNodeMapper::iterator Region_iter = Graph.Region_Nodes_Map.begin(); Region_iter != Graph.Region_Nodes_Map.end(); Region_iter++){
		os << "   Region "<< (*Region_iter).first  <<": Number of Nodes: "<< (*Region_iter).second->nodes_inside.size() << "\n";
		if ((*Region_iter).second->id != -1)
			os << "        Center "<<  (*Region_iter).second->Node_Center->info.position << "\n";
		os << "        Subgraphs "<<  (*Region_iter).second->sub_graphs.size() << "\n";
		for( std::list < std::list <Node*> >::iterator it = (*Region_iter).second->sub_graphs.begin();it != (*Region_iter).second->sub_graphs.end(); it++ ){
			os << "        Subgraphs size "<<  (*it).size() << " nodes" <<"\n";
		}
		os << "   Connections "<< "\n";
		for(int i=0; i<  (*Region_iter).second->connected.size(); i++){
			std::set <int> link = (*Region_iter).second->connected[i]->Nodes_ids;
			os << "     ("<< (*link.begin()) <<","<<  (*link.rbegin()) <<  ")  " ;			
		}
		os << "\n";			

	}
	/////////////
	os << "Region Edges "<< "\n";
	for (RegionEdgeMapper::iterator Region_Edge_iter = Graph.Region_Edges_Map.begin(); Region_Edge_iter != Graph.Region_Edges_Map.end(); Region_Edge_iter++){
		std::set<int> link = (*Region_Edge_iter).second->Nodes_ids;
		os << "    Path ("<< (*link.begin()) <<","<<  (*link.rbegin()) <<  ")  has " << (*Region_Edge_iter).second->Edges_in_border.size()  << " connections" << "\n";	
	}
    return os;
}


//////////////////////////////



std::vector<float> RegionGraph::extract_error_per_node(std::vector<geometry_msgs::Point>  gt_nodes, float & average){
	std::vector<float> error_vector;
//	if(gt_nodes.size() == current_node_id+1){
	if(true){
		float cum_error=0;
		for(int i=0; i < gt_nodes.size();i++){
			std::complex<double> gt_current(gt_nodes[i].x, gt_nodes[i].y );
			float difference = abs(gt_current - Nodes_Map[i]->info.position);
			error_vector.push_back(difference);
			cum_error += difference;
//			std::cout <<"Node "<< i <<", GT"<<gt_current  <<",  node id "<< Nodes_Map[i]->info.position <<", Error "<< difference  <<std::endl;
		}
//		std::cout <<"Node "<< current_node_id <<",  node id "<< Nodes_Map[current_node_id]->info.position << std::endl;
		average= cum_error/gt_nodes.size();
	}
	else{
		error_vector.push_back(-1);

	}
	return error_vector;		
}





int RegionGraph::build_Region_Graph(std::vector<geometry_msgs::Point> edge_markers, nav_msgs::MapMetaData info, cv::Mat  Tag_image, cv::Mat  original_image){
	
	image_info = info;
	int number_of_edges = edge_markers.size()/2;				
	
	Node* from_Node_ptr; 
	Node* TO_Node_ptr;   
		
	std::complex<double> origin(info.origin.position.x, info.origin.position.y);	
	
	
	Region_Node* Unexplored_region = new Region_Node;
	Unexplored_region->id=-1;
	Region_Nodes_Map[-1] = Unexplored_region;	
	
	current_node_id = 0;
	
	for (int i=0; i < number_of_edges;i++){
		std::complex<double> FROM_position( edge_markers[2*i].x , edge_markers[2*i].y  );
		std::complex<double> TO_position  (edge_markers[2*i+1].x, edge_markers[2*i+1].y);

		int FROM_label = edge_markers[2*i].z*1000;
		int TO_label   = edge_markers[2*i+1].z*1000;

		current_node_id = std::max(current_node_id , FROM_label);
		current_node_id = std::max(current_node_id , TO_label);

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
					current_region->id = region_tag;
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
					current_region->id = region_tag;
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

//	std::cout << "   Everything inserted "<< std::endl;

	extract_subgraph();
//	std::cout << "   Subgraph extracted "<< std::endl;

	build_region_graph(Tag_image, original_image);
//	std::cout << "   Region Graph Built "<< std::endl;
	
	find_edges_between_regions();
//	std::cout << "   Edges Between Regions Added "<< std::endl;

	find_center_of_regions();
//	std::cout << "   Center of Regions Found "<< std::endl;

//	segment_frontier (Nodes_Map[current_node_id]->info.region_label, Tag_image );
/*
	std::set<int> touching_regions;
	touching_regions.insert(-1);
	touching_regions.insert(Nodes_Map[current_node_id]->info.region_label);
	
	RegionEdgeMapper::iterator reg_it = Region_Edges_Map.find(touching_regions);
	if(reg_it != Region_Edges_Map.end() ){
		segment_edge (touching_regions, Tag_image );
	}
*/
//	segment_every_edge (Nodes_Map[current_node_id]->info.region_label, Tag_image);
	segment_every_edge (-1 , original_image);
	segment_every_edge (Nodes_Map[current_node_id]->info.region_label, original_image);


	


	///////
	return 1;
}

/////////////////


void RegionGraph::extract_subgraph(){
	
	for (RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.begin(); Region_iter != Region_Nodes_Map.end(); Region_iter++){
		std::list <Node*> Remaining_Nodes = (*Region_iter).second->nodes_inside, Nodes_in_sub_Region;
		int sub_region=0;

		while( Remaining_Nodes.size() > 0){// Breadth first
			evaluate_list_connectivity(Remaining_Nodes, sub_region);
			Remaining_Nodes.clear();
			Nodes_in_sub_Region.clear();

			for(Node_iter it = (*Region_iter).second->nodes_inside.begin(); it != (*Region_iter).second->nodes_inside.end(); it++){		
				if( (*it)->info.sub_region == -1 ){
					Remaining_Nodes.push_back(*it);
				}
				else if( (*it)->info.sub_region == sub_region ){
					Nodes_in_sub_Region.push_back(*it);
				}
			}
			sub_region++;
			(*Region_iter).second->sub_graphs.push_back(Nodes_in_sub_Region);
		}
		////		
	}
}
//////////
void RegionGraph::evaluate_list_connectivity(std::list <Node*> list_in, int name){

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


int RegionGraph::number_loop_closures(){
	int number=0;
	
	for (std::unordered_map <int,Edge*>::iterator map_iter = Edges_Map.begin(); map_iter != Edges_Map.end(); map_iter++  ){
		int first_node  = (*map_iter).second->from->info.label;
		int second_node = (*map_iter).second->to->info.label;
		
		if (second_node - first_node != 1){
			number++;
		}
		//
	}
	
	return number;
}





void RegionGraph::build_region_graph(cv::Mat  Tag_image, cv::Mat  original_image){
	
	int window_size=1;
	
	
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
			if (connections_in_region.size()==2 || (frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ) ){
				mapping_region_to_point_array[center_tag-1].push_back(window_center);
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
	
	// std::cout << "      Image processed "<< std::endl;
	
	for (region_points_mapper::iterator it2 = mapping_region_to_point_array.begin(); it2 != mapping_region_to_point_array.end(); it2 ++){
		int region_tag = (*it2).first;		
		RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
		if(Region_iter == Region_Nodes_Map.end() ){// not found
			Region_Node* current_region = new Region_Node;
			current_region->id = region_tag;
			Region_Nodes_Map[region_tag] = current_region;				
		}				
		Region_Nodes_Map[region_tag]->contour = (*it2).second;		
	}
	//	std::cout << "      Contour Extracted "<< std::endl;
		


	for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){
		if(it2->second.size() > 10){ //Avoids sparse connections
			Region_Edge *InsideEdge;
			InsideEdge = new Region_Edge;
			
			InsideEdge->frontier = it2->second;
			std::set<int> conection =it2->first;
			InsideEdge->First_Region  = Region_Nodes_Map[ (*conection.begin()) ];
			InsideEdge->Second_Region = Region_Nodes_Map[ (*conection.rbegin()) ];
			InsideEdge->Nodes_ids = conection;
			
			Region_Nodes_Map[ (*conection.begin()) ]->connected.push_back(InsideEdge);
			Region_Nodes_Map[ (*conection.rbegin()) ]->connected.push_back(InsideEdge);
			
			Region_Edges_Map[conection] =InsideEdge;
		}
				
	}
	// std::cout << "      Edge Extracted "<< std::endl;

	for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){
		if(it2->second.size() > 0){
			Region_Edge *InsideEdge;
			InsideEdge = new Region_Edge;
			
			InsideEdge->frontier = it2->second;
			std::set<int> conection =it2->first;
			InsideEdge->First_Region  = Region_Nodes_Map[ (*conection.begin()) ];
			InsideEdge->Second_Region = Region_Nodes_Map[ (*conection.rbegin()) ];
			InsideEdge->Nodes_ids = conection;
			
			Region_Nodes_Map[ (*conection.begin()) ]->connected.push_back(InsideEdge);
			Region_Nodes_Map[ (*conection.rbegin()) ]->connected.push_back(InsideEdge);
			
			Region_Edges_Map[conection] =InsideEdge;	
		}
	}
	// std::cout << "      Frontier Extracted "<< std::endl;


}



void RegionGraph::find_edges_between_regions(){
	std::list <Edge*> connecting_edges;
		
	for(EdgeMapper::iterator it = Edges_Map.begin(); it != Edges_Map.end(); it++){

		int region_from = (*it).second->from->info.region_label;
		int region_to   = (*it).second->to->info.region_label;
		
		std::set<int> connection;
		connection.insert(region_from);
		connection.insert(region_to);
			
		
		if( region_from != region_to ){
			RegionEdgeMapper::iterator REM_iter;
			REM_iter = Region_Edges_Map.find(connection);
			if (REM_iter != Region_Edges_Map.end()){
				(*REM_iter).second->Edges_in_border.push_back( (*it).second );
			}
		}
		
	}

}


void RegionGraph::find_center_of_regions(){

	for (RegionNodeMapper::iterator reg_it = Region_Nodes_Map.begin(); reg_it != Region_Nodes_Map.end();reg_it ++){
		Region_Node* current_region = (*reg_it).second;
		
		if (current_region->id != -1){
			// Find center of nodes
			std::complex<double> cum_position (0,0);
			/*
			for (std::list <Node*>::iterator node_iter = current_region->nodes_inside.begin(); node_iter != current_region->nodes_inside.end(); node_iter++){
				cum_position += (*node_iter)->info.position;
			} 
			cum_position /= (double)Region_Nodes_Map.size();
			//*/
			for(int i=0; i < current_region-> contour.size(); i++){
				cv::Point current_point = current_region-> contour[i];
				double x = current_point.x * image_info.resolution + image_info.origin.position.x;
				double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
		
				std::complex<double> transformed_point(x,y);
				cum_position += transformed_point;
			}
			cum_position /= (double)current_region-> contour.size();


			
			//Find the node closest to the center
			float min_dist = std::numeric_limits<float>::infinity();
			Node* min_node=*( current_region->nodes_inside.begin() );
			for (std::list <Node*>::iterator node_iter = current_region->nodes_inside.begin(); node_iter != current_region->nodes_inside.end(); node_iter++){
				float norm = std::norm(cum_position - (*node_iter)->info.position);
				if(norm < min_dist){
					min_dist = norm;
					min_node = *node_iter;
				}
			} 		
			//Assign that node
			current_region->Node_Center = min_node;
			
		
		}
	}
}



void RegionGraph::segment_every_edge (int region_id, cv::Mat  original_image){
//	Region_Node* current_region = Region_Nodes_Map[Nodes_Map[current_node_id]->info.region_label];
	Region_Node* current_region = Region_Nodes_Map[region_id];
	
	
	for( std::vector<Region_Edge*>::iterator region_edge_iter = current_region->connected.begin(); region_edge_iter != current_region->connected.end();region_edge_iter++){
		segment_edge ( (*region_edge_iter)->Nodes_ids , original_image);
	}

	
	
}


//void RegionGraph::segment_frontier (int region_id, cv::Mat  Tag_image){
cv::Mat RegionGraph::segment_edge (std::set<int> edge_region_index, cv::Mat  original_image){
	cv::Mat frontier_image = cv::Mat::zeros(original_image.size(), CV_8U);
	cv::Mat obstacle_image = (original_image > 75) & (original_image<101);
	
	cv::dilate(obstacle_image, obstacle_image, cv::Mat(),    cv::Point(-1,-1) , 7 );
	
	/*
	std::set < int > edge_region_index;
	edge_region_index.insert(-1);
	edge_region_index.insert(region_id);
	*/
//	std::cout <<  "Region index ["<< *(edge_region_index.begin() ) << ","<<*(edge_region_index.rbegin() ) << "]" <<std::endl;
	
	Region_Edge* current_region_edge = Region_Edges_Map[edge_region_index];
	std::vector<cv::Point> frontier_points = current_region_edge->frontier;
	
	
	for(int i=0; i < frontier_points.size();i++ ){
		frontier_image.at<uchar>(frontier_points[i])=255;
	}

	frontier_image &= ~obstacle_image;
//	cv::dilate(frontier_image, frontier_image,cv::Mat(),    cv::Point(-1,-1) , 2 );
	cv::Mat destroyable = frontier_image.clone();
		
	std::vector<std::vector<cv::Point> > contours, contours_cleaned;
	std::vector<cv::Vec4i> hierarchy;
	
	cv::findContours( destroyable, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
	
	for(int i=0; i< contours.size();i++){
		if(contours[i].size()>10){
			contours_cleaned.push_back(contours[i]);
			
			Region_Sub_Edge new_sub_edge;
			new_sub_edge.frontier = contours[i];
			current_region_edge->Sub_Edges.push_back(new_sub_edge);
			
		}
	}

//	std::cout <<  "   Different Frontiers: "<< contours_cleaned.size() << std::endl;	
	current_region_edge->segmented_frontier = contours_cleaned;

	return frontier_image;

}


cv::Mat RegionGraph::segment_current_frontier ( cv::Mat  Tag_image){
	std::set<int> edge_to_segment = {-1, Nodes_Map[current_node_id]->info.region_label};
	
	RegionEdgeMapper::iterator reg_iter = Region_Edges_Map.find(edge_to_segment);
	if (reg_iter != Region_Edges_Map.end() ){
		return segment_edge (edge_to_segment, Tag_image);
	}
	else{
		return Tag_image;
	}
	
}




std::vector<std::complex<double> > RegionGraph::collect_frontiers(){
	int a=1;
	std::vector<std::complex<double> > points_in_edges;
	
	Region_Node* current_region = Region_Nodes_Map[Nodes_Map[current_node_id]->info.region_label];
	
	for(std::vector<Region_Edge*>::iterator edge_iter = current_region->connected.begin(); edge_iter != current_region->connected.end(); edge_iter++){
		Region_Edge* current_edge = *edge_iter;
		std::vector<cv::Point> pixel_frontier = current_edge->frontier;

		for(std::vector<cv::Point>::iterator point_iter = pixel_frontier.begin(); point_iter != pixel_frontier.end(); point_iter++){
			cv::Point current_point = *point_iter;
			double x = current_point.x * image_info.resolution + image_info.origin.position.x;
			double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;

			std::complex<double> transformed_point(x,y);
			points_in_edges.push_back(transformed_point);			
		}
	}
	
	return points_in_edges;
}

std::vector<std::complex<double> > RegionGraph::collect_all_frontiers(){
	
	std::vector<std::complex<double> > points_in_edges;
	
//	Region_Node* current_region = Region_Nodes_Map[Nodes_Map[current_node_id]->info.region_label];

	for(RegionNodeMapper::iterator node_iter = Region_Nodes_Map.begin();  node_iter != Region_Nodes_Map.end(); node_iter++){
		Region_Node* current_region = (*node_iter).second;
		if(current_region->id != -1){
			points_in_edges.push_back( current_region->Node_Center->info.position  );
			for(std::vector<Region_Edge*>::iterator edge_iter = current_region->connected.begin(); edge_iter != current_region->connected.end(); edge_iter++){
				Region_Edge* current_edge = *edge_iter;
				std::vector<cv::Point> pixel_frontier = current_edge->frontier;
				/*
				for(std::vector<cv::Point>::iterator point_iter = pixel_frontier.begin(); point_iter != pixel_frontier.end(); point_iter++){
					cv::Point current_point = *point_iter;
					double x = current_point.x * image_info.resolution + image_info.origin.position.x;
					double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
			
					std::complex<double> transformed_point(x,y);
					points_in_edges.push_back(transformed_point);			
				}
				// */ 
				std::vector< std::vector< cv::Point> > pixel_frontier_segmented = current_edge->segmented_frontier;
				for(int i=0; i<pixel_frontier_segmented.size();i++){
					
					int median_index = floor(pixel_frontier_segmented[i].size()/4);
					
					cv::Point current_point = pixel_frontier_segmented[i][median_index];
					double x = current_point.x * image_info.resolution + image_info.origin.position.x;
					double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
			
					std::complex<double> transformed_point(x,y);
					points_in_edges.push_back(transformed_point);			
					
				}
				
			}
		}
		
	}



	return points_in_edges;
}


geometry_msgs::PoseStamped RegionGraph::extract_exploration_goal( std::vector<int> input_regions){

	std::vector<std::complex<double> > exploration_goals;
	int choices_sizes = input_regions.size();
	
	geometry_msgs::PoseStamped goal_to_publish;


	switch(choices_sizes){
		case 0:
			std::cout << " No choices, check map" << std::endl;
			check_map( goal_to_publish );
			
			break;
			
		case 1:
			std::cout << " Only one choice! " << std::endl;
			
			goal_to_publish = choose_closer_frontier(input_regions);

			break;
			
		default:
			std::cout << " Multiple choices, eliminate frontier and choose closer" << std::endl;
			// Eliminate unexplored regions
			std::vector<int>::iterator iter = std::find(input_regions.begin(), input_regions.end(), -1) ;			
			if(iter != input_regions.end()){
				input_regions.erase(iter);
			}
			int rand_index = std::rand() % (input_regions.size());
			//Choose random
//			std::cout << " chosen is: " << input_regions[rand_index]<< std::endl;			

			goal_to_publish = choose_closer_frontier(input_regions);
			
			break;				
		}
		return goal_to_publish;
		////
}

geometry_msgs::PoseStamped RegionGraph::choose_closer_frontier(std::vector<int> region){
	int current_region_id = Nodes_Map[current_node_id]->info.region_label;

	geometry_msgs::PoseStamped pose_out;

	std::set<int> current_edge_frontier ={current_region_id};
	std::complex<double> current_pos = Nodes_Map[current_node_id]->info.position;
	
	
	if(region[0] == -1){		// choose frontier
		current_edge_frontier.insert(region[0]);
		
		std::vector < std::vector<cv::Point> > pixel_frontier_segmented = Region_Edges_Map[current_edge_frontier]->segmented_frontier ;	
		std::vector < std::complex<double> > points_in_edges;
		//Choose mean points in edges
		for(int i=0; i < pixel_frontier_segmented.size();i++){			
			// Choose median
//			int median_index = floor(pixel_frontier_segmented[i].size()/4);			
//			cv::Point current_point = pixel_frontier_segmented[i][median_index];
			
			//Choose center
			cv::Moments mu = cv::moments(pixel_frontier_segmented[i]);
			cv::Point current_point( mu.m10/mu.m00 , mu.m01/mu.m00 );
			
			double x = current_point.x * image_info.resolution + image_info.origin.position.x;
			double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
	
			std::complex<double> transformed_point(x,y);
			points_in_edges.push_back(transformed_point);			
			
		}
		// Choosing the closest to current id

		std::complex<double> closest_pos;
		float min_dist = std::numeric_limits<float>::infinity();
		double angle=0;
		
		for(int i=0; i< points_in_edges.size();i++){
			std::complex<double> difference = points_in_edges[i] - current_pos;
			if (abs(difference) < min_dist){
				min_dist = abs(difference);
				closest_pos = points_in_edges[i];
				angle = arg(difference);
			}
		}
		
		pose_out = construct_msg(closest_pos,  angle) ;		
	}
	
	else{
		//// Checking all nodes
		float min_dist = std::numeric_limits<float>::infinity();
		int min_index = region[0];
		std::complex<double> min_position;
		double min_angle = 0;
		
		for (int i=0; i < region.size(); i++){
			std::complex<double> current_position;
			double current_angle ;
			////
			if( Region_Nodes_Map[region[i]]->nodes_inside.size() > 1 ){ // Choose  center node
				Node* node_to_explore = Region_Nodes_Map[region[i] ]->Node_Center;
//				double angle;
				std::complex<double> node_position = node_to_explore->info.position;
	
				if (node_to_explore->info.label != 0 ){
					Node* previous_node = Nodes_Map[node_to_explore->info.label -1 ];
					current_angle = arg(node_position  -  previous_node->info.position );
				}
				else{// initial node 
					current_angle=0;
				}
				current_position = node_position;
				
//				return construct_msg(node_position,  angle);		
	
			}
			////			
			else{// No nodes inside choose geometric center
				//find geometric center
				Region_Node* current_region = Region_Nodes_Map[region[i] ];
				std::complex<double> cum_position(0,0);
				for(int j=0; j < current_region-> contour.size(); j++){
					cv::Point current_point = current_region-> contour[j];
					double x = current_point.x * image_info.resolution + image_info.origin.position.x;
					double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
			
					std::complex<double> transformed_point(x,y);
					cum_position += transformed_point;
				}
				cum_position /= (double)current_region-> contour.size();
		
				current_angle = arg(cum_position - Nodes_Map[current_node_id]->info.position  );			
				
				current_position = cum_position;
//				return construct_msg(cum_position,  angle);		
			}
			///////
			
			float distance = abs(current_position - current_pos);
			if(distance < min_dist){
				min_position = current_position;
				min_angle = current_angle;
				min_index = region[i];
			}
			////
		}
		std::cout << "  Chosen region is " <<  min_index << std::endl;
		pose_out = construct_msg(min_position,  min_angle) ;			
	}
	return pose_out;
}



int RegionGraph::check_map( geometry_msgs::PoseStamped& pose_msg ){	
	int ready = -1;
	Region_Node* unexplored_region = Region_Nodes_Map[-1];
	pose_msg.pose.orientation.w = 0; //bad formed quaternion



	
			
	std::vector<Region_Edge*> region_frontiers = unexplored_region->connected;
//	std::cout << "Other frontiers size " << region_frontiers.size() << std::endl;
	
//	std::cout << "Checking map "  << std::endl;
	int sum_of_points = 0;
	for(int i=0;i<region_frontiers.size();i++){
		int points_in_frontier = region_frontiers[i]->segmented_frontier.size();
		sum_of_points += points_in_frontier;
//		std::cout << "  Frontier ("  << *(region_frontiers[i]->Nodes_ids.begin() ) <<","<< *(region_frontiers[i]->Nodes_ids.rbegin() ) << ") has "<< points_in_frontier  << " points"<<std::endl;		
		
	}
	
	
//	if(region_frontiers.size() == 0){
	if(sum_of_points == 0){
		std::cout << "No other frontiers "  << std::endl;
		ready = 1;
	}
	else{
		std::vector < std::complex<double> > points_in_edges;
		
		for(int i=0; i < region_frontiers.size();i++){
			std::vector < std::vector<cv::Point> > pixel_frontier_segmented = region_frontiers[i]->segmented_frontier ;
			
			//Choose mean points in edges
			for(int i=0; i < pixel_frontier_segmented.size();i++){			
				//Choose center
				cv::Moments mu = cv::moments(pixel_frontier_segmented[i]);
				cv::Point current_point( mu.m10/mu.m00 , mu.m01/mu.m00 );
				
				double x = current_point.x * image_info.resolution + image_info.origin.position.x;
				double y = (image_info.height - current_point.y) * image_info.resolution + image_info.origin.position.y;
		
				std::complex<double> transformed_point(x,y);
				points_in_edges.push_back(transformed_point);						
			}
		}
//		std::cout << "Segmented frontier size " << points_in_edges.size() << std::endl;
		


		// Choosing the closest to current id

		std::complex<double> closest_pos;
		float min_dist = std::numeric_limits<float>::infinity();
		double angle=0;
		std::complex<double> current_pos = Nodes_Map[current_node_id]->info.position;
		
		for(int i=0; i< points_in_edges.size();i++){
			std::complex<double> difference = points_in_edges[i] - current_pos;
			if (abs(difference) < min_dist){
				min_dist = abs(difference);
				closest_pos = points_in_edges[i];
				angle = arg(difference);
			}
		}
		
		pose_msg = construct_msg(closest_pos,  angle) ;		
		
		
	}
	////
	return ready;
}


//geometry_msgs::PoseStamped RegionGraph::Tremaux_data( ){	
int RegionGraph::Tremaux_data( geometry_msgs::PoseStamped& pose_msg ){	
	
	
	std::cout << "Current Node id is  "<< current_node_id << std::endl;
	Region_Node* current_Region = Region_Nodes_Map[ Nodes_Map[current_node_id]->info.region_label ];

	std::cout << "Current Region is  "<< current_Region->id << std::endl;
	
	int region_completed = 1;
	
	// Is it fully connected?
	if (current_Region->sub_graphs.size() >= 2 ){
		std::cout << "   Number of subgraphs  "<< current_Region->sub_graphs.size() << ", Should connect region graph  " << std::endl;
		region_completed = -1;
	}
	
	std::map<int, std::set< Edge*> > markers_in;
	std::map<int, std::set< Edge*> > markers_out;


	std::cout << "Paths  " << std::endl;
	for( std::vector<Region_Edge*>::iterator region_edge_iter = current_Region->connected.begin(); region_edge_iter != current_Region->connected.end(); region_edge_iter ++){
		std::set<int> connections =  (*region_edge_iter)->Nodes_ids;
		int connected_region;
		if (*(connections.begin() ) == current_Region->id){
			std::cout << "  ( " << *(connections.begin() )<< " , "<< *(connections.rbegin() )  << " )"<< std::endl;
			connected_region = *(connections.rbegin() );
		}
		else{
			std::cout << "  ( " << *(connections.rbegin() )<< " , "<< *(connections.begin() )  << " )"<< std::endl;
			connected_region = *(connections.begin() );
		} 	
		
		if( (*region_edge_iter)->Edges_in_border.size() == 0 ){
			std::cout << "     no link "<< std::endl;
			std::set< Edge*> empty_set;
			markers_out[connected_region] = empty_set;
			markers_in[connected_region] = empty_set;
		}

//		std::cout << "  connected_region " <<  connected_region << std::endl;

		std::set< Edge*> empty_set;
		markers_out[connected_region] = empty_set;
		markers_in[connected_region]  = empty_set;

		for(std::vector <Edge*>::iterator edge_iter =  (*region_edge_iter)->Edges_in_border.begin(); edge_iter !=  (*region_edge_iter)->Edges_in_border.end(); edge_iter++  ){
			Edge *current_edge = *edge_iter;

			int region_from = current_edge->from->info.region_label;
			int region_to   = current_edge->to->info.region_label;
		//	std::cout << "     from " <<  region_from  << " to "<< region_to;
			
			if( region_from == current_Region->id){
		//		std::cout << ", link out " <<  std::endl;
				markers_out[connected_region].insert(*edge_iter);

			}
			else{
		//		std::cout << ", link in " <<  std::endl;
				markers_in[connected_region].insert(*edge_iter);
			}
			
		}
	

		
	}
	


	///////////////////////////	
	std::cout << "  Regions with path "<< std::endl;
	std::map<int, std::vector<int> > regions_per_marks;
	
	
	for( std::map<int, std::set< Edge*> >::iterator  marker_iter = markers_in.begin(); marker_iter != markers_in.end(); marker_iter ++){
		int number_of_marks = 0;
		int current_region = (*marker_iter).first;
		std::cout << "     Region "<< current_region << ":";			
		
		if(markers_in[current_region].size() > 0){
			std::cout << "  marked in";	
			number_of_marks++;		
		}
		if( markers_out[current_region].size() > 0){
			std::cout << "  marked out";			
			number_of_marks++;
		}
		std::cout<< ", Total Marks: "<< number_of_marks << std::endl;
		regions_per_marks[number_of_marks].push_back(current_region);
				
	}
	
	std::vector<int> regions_to_explore;
	
	if( regions_per_marks[0].size() > 0){
		std::cout<< "Nodes to explore with no mark: ";
		regions_to_explore = regions_per_marks[0];
		for(std::vector<int>::iterator int_iter = regions_per_marks[0].begin(); int_iter != regions_per_marks[0].end(); int_iter ++ ){
			std::cout<< *int_iter<<"    ";
		}
		std::cout << std::endl;
	}
	else if(regions_per_marks[1].size() > 0){
		std::cout<< "Nodes to explore with no mark out: ";
		for(std::vector<int>::iterator int_iter = regions_per_marks[1].begin(); int_iter != regions_per_marks[1].end(); int_iter ++ ){
			if( markers_out[*int_iter].size() == 0){
				std::cout<< *int_iter<<"    ";
				regions_to_explore.push_back( *int_iter );
			}
		}
		std::cout << std::endl;
	}
	else{
		std::cout<< "Region Ready "<< std::endl;
		region_completed=0;
	}
	//////////////
	pose_msg = extract_exploration_goal (regions_to_explore);
//	return(extract_exploration_goal (regions_to_explore) );
	return( region_completed );
		
	////////////////////////////


}




int RegionGraph::connect_inside_region( geometry_msgs::PoseStamped& pose_msg ){
	
	int status = 1;
	float distance_threshold = 0.05;
	std::cout << " Trying to connect inside" << std::endl;		
	Region_Node* current_Region = Region_Nodes_Map[  Nodes_Map[current_node_id]->info.region_label   ];
	
	float min_distance = std::numeric_limits<float>::infinity();
	
	std::list < std::list <Node*> > subGraphs = current_Region->sub_graphs;
	int region_number=0;
	int current_subgraph = Nodes_Map[current_node_id]->info.sub_region;
//	std::complex<double> first_position;// = Nodes_Map[current_node_id]->info.position;
	
	// Find current subgraph iterator
	std::list <Node*>  current_list;
	for(std::list < std::list <Node*> >::iterator graph_list_iter = current_Region->sub_graphs.begin(); graph_list_iter != current_Region->sub_graphs.end(); graph_list_iter ++){
		int current_sub_region = (*( (*graph_list_iter).begin() ))->info.sub_region;
		if(current_sub_region == current_subgraph ){
			current_list = *graph_list_iter;
			break;
		}
	}
	//////

	std::map <int, float> min_dist_mapper;
	std::map <int, Node*> min_Node_mapper;

/*
	// Compare current region with the regions
	for(std::list <Node*>::iterator node_iter = current_list.begin(); node_iter != current_list.end(); node_iter++){
		std::complex<double> first_position = (*node_iter)->info.position;
	}
	*/
	
	for(std::list < std::list <Node*> >::iterator graph_list_iter = current_Region->sub_graphs.begin(); graph_list_iter != current_Region->sub_graphs.end(); graph_list_iter ++){
		int current_sub_region = (*( (*graph_list_iter).begin() ))->info.sub_region;
	
		if( current_sub_region != current_subgraph){
//			std::cout << "   analizing sub region " <<  current_sub_region  ;

			// Calculate minimum
			float min_subgraph_distance = std::numeric_limits<float>::infinity();			
			Node* current_Node_min = *( (*graph_list_iter).begin() );

			for(std::list <Node*>::iterator node_iter = current_list.begin(); node_iter != current_list.end(); node_iter++){
				std::complex<double> first_position = (*node_iter)->info.position;			
				
				for(std::list <Node*>::iterator second_node_iter = (*graph_list_iter).begin(); second_node_iter != (*graph_list_iter).end(); second_node_iter++){
					std::complex<double> second_position = (*second_node_iter)->info.position;
					
					float current_distance = norm(first_position - second_position  );
					if(current_distance < min_subgraph_distance){
						min_subgraph_distance = current_distance;
						current_Node_min = *second_node_iter;
					}				
				}
			}

			if (min_subgraph_distance < distance_threshold){
//				std::cout << ", Considered connected "  << std::endl;			
			}
			else{
				min_dist_mapper[current_sub_region] = min_subgraph_distance;
				min_Node_mapper[current_sub_region] = current_Node_min;
//				std::cout << ", with distance: " <<  min_subgraph_distance  << std::endl;
			}
			
		}
	}		

	if(min_dist_mapper.size() == 0){
		std::cout << "   region considered connected " <<  std::endl;
		status = -1;
	}
	else{
		// Calculate minimum
		float min_subgraph_distance = std::numeric_limits<float>::infinity();			
		int min_index;
		
		for( std::map <int, float>::iterator map_iter = min_dist_mapper.begin(); map_iter != min_dist_mapper.end();map_iter++){
			if( (*map_iter).second < min_subgraph_distance){
				min_index = (*map_iter).first;
				min_subgraph_distance = (*map_iter).second;
			}
		}

		{
			double current_angle ;		
			Node* node_to_explore = min_Node_mapper[min_index];
			std::complex<double> node_position = node_to_explore->info.position;
	
			if (node_to_explore->info.label != 0 ){
				Node* previous_node = Nodes_Map[node_to_explore->info.label -1 ];
				current_angle = arg(node_position  -  previous_node->info.position );
			}
			else{// initial node 
				current_angle=0;
			}
			pose_msg = construct_msg(node_position,  current_angle) ;
		}

		
		
	}
	
	
	////////
	return status;
}


int RegionGraph::connect_inside_region_greedy( geometry_msgs::PoseStamped& pose_msg ){
	
	int status = 1;
	std::cout << " Trying to connect inside" << std::endl;		
	Region_Node* current_Region = Region_Nodes_Map[  Nodes_Map[current_node_id]->info.region_label   ];
	
	float min_distance = std::numeric_limits<float>::infinity();
	
	std::list < std::list <Node*> > subGraphs = current_Region->sub_graphs;
	int region_number=0;
	int current_subgraph = Nodes_Map[current_node_id]->info.sub_region;
	std::complex<double> first_position = Nodes_Map[current_node_id]->info.position;
	
	
	std::cout << " connect sub-graph " << current_subgraph << std::endl;

	std::map <int, float> min_dist_mapper;
	std::map <int, Node*> min_Node_mapper;


	// Compare current region with the regions

	for(std::list < std::list <Node*> >::iterator graph_list_iter = current_Region->sub_graphs.begin(); graph_list_iter != current_Region->sub_graphs.end(); graph_list_iter ++){
		int current_sub_region = (*( (*graph_list_iter).begin() ))->info.sub_region;
	
		if( current_sub_region != current_subgraph){
			std::cout << "   analizing sub region " <<  current_sub_region  ;

			// Calculate minimum
			float min_subgraph_distance = std::numeric_limits<float>::infinity();			
			Node* current_Node_min = *( (*graph_list_iter).begin() );
			
			for(std::list <Node*>::iterator second_node_iter = (*graph_list_iter).begin(); second_node_iter != (*graph_list_iter).end(); second_node_iter++){
				std::complex<double> second_position = (*second_node_iter)->info.position;
				
				float current_distance = norm(first_position - second_position  );
				if(current_distance < min_subgraph_distance){
					min_subgraph_distance = current_distance;
					current_Node_min = *second_node_iter;
				}				
			}
			if (min_subgraph_distance < 0.15){
				std::cout << ", Considered connected "  << std::endl;			
			}
			else{
				min_dist_mapper[current_sub_region] = min_subgraph_distance;
				min_Node_mapper[current_sub_region] = current_Node_min;
				std::cout << ", with distance: " <<  min_subgraph_distance  << std::endl;
			}
			
		}
	}		
	
	if(min_dist_mapper.size() == 0){
		std::cout << "   region considered connected " <<  std::endl;
		status = -1;
	}
	else{
		// Calculate minimum
		float min_subgraph_distance = std::numeric_limits<float>::infinity();			
		int min_index;
		
		for( std::map <int, float>::iterator map_iter = min_dist_mapper.begin(); map_iter != min_dist_mapper.end();map_iter++){
			if( (*map_iter).second < min_subgraph_distance){
				min_index = (*map_iter).first;
				min_subgraph_distance = (*map_iter).second;
			}
		}
		std::cout << "Choosing position " << min_Node_mapper[min_index]->info.position  << std::endl;
		{
			double current_angle ;		
			Node* node_to_explore = min_Node_mapper[min_index];
			std::complex<double> node_position = node_to_explore->info.position;
	
			if (node_to_explore->info.label != 0 ){
				Node* previous_node = Nodes_Map[node_to_explore->info.label -1 ];
				current_angle = arg(node_position  -  previous_node->info.position );
			}
			else{// initial node 
				current_angle=0;
			}
			pose_msg = construct_msg(node_position,  current_angle) ;
		}

		
		
	}
	
	
	////////
	return status;
}



/*
Tremaux

*path is unvisited, marked once or twice

1. Remember direction taken

2. Is noded marked (has been visited)?
  2.1 Not marked -> choose a random direction and mark
  2.2 Is marked,
     2.2.1  if direction is marked once return the same direction
     2.2.2  else, pick direction with fewest marks (random if more than once)

*/




