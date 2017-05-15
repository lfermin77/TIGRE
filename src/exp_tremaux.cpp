//ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"


//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//Graph
#include "graph.hpp"

// Standard
#include "time.h"
#include <fstream>



class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Subscriber graph_sub_;	
	ros::Subscriber Uncertainty_sub_;
	ros::Subscriber trajectory_sub_;
	ros::Subscriber ground_truth_sub_;
	
    ros::Publisher  pose_array_pub_;	
    ros::Publisher  markers_pub_;	
    ros::Publisher  goal_pub_;	
	ros::Timer timer;
	
	nav_msgs::MapMetaData map_info;				
	std::vector<geometry_msgs::Point> edges;	
	geometry_msgs::Point Last_node;
	int counter;
	
	cv::Mat image_map, image_tagged;
	
	bool map_received, path_received, graph_received, tagged_image_received, GT_received;
	geometry_msgs::PoseStamped pose_to_publish; 
	geometry_msgs::PoseStamped Last_goal; 

	//Statistics
	float distance;
	float cum_time;
	float time_counter;	
	std::ofstream myfile;
	ros::Subscriber save_sub_;	
	
	std::vector <geometry_msgs::Point> ground_truth;
	geometry_msgs::Point current_ground_truth;
	geometry_msgs::Point *first_pose;
	
	public:
		ROS_handler(const std::string& mapname) : mapname_(mapname),  it_(n)
		{
			ROS_INFO("Waiting for the map");
			map_sub_	 		= n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			trajectory_sub_ 	= n.subscribe("trajectory", 1, &ROS_handler::trajectoryCallback, this);			
			ground_truth_sub_ 	= n.subscribe("GT_poses", 1, &ROS_handler::ground_truth_Callback, this);			
			graph_sub_ 			= n.subscribe("SLAM_Graph", 10, &ROS_handler::graphCallback, this);			

			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/processed_image", 1);
			image_sub_ = it_.subscribe("tagged_image", 1,  &ROS_handler::imageCallback, this);
			
			markers_pub_     =  n.advertise<visualization_msgs::Marker>( "Exploration_Choices", 10 );
			
			cv_ptr.reset (new cv_bridge::CvImage);
//			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			goal_pub_	  	 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
//			goal_pub_	  	 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_2", 10);
			
			counter =0;
			map_received = path_received = graph_received = tagged_image_received = GT_received = false;
			
			//Statistics
			save_sub_ = n.subscribe("save_file", 10, &ROS_handler::UncertaintyCallback, this);
			Last_goal.header.seq = -1;
			distance = 0;
			time_counter = 0;
			cum_time=0;
			first_pose=NULL;
			
			std::string path_exploration =ros::package::getPath ("tigre");
			
			path_exploration += "/results/Results.txt";

			std::cout << "Path "<< path_exploration << std::endl<< std::endl<< std::endl;
			
			myfile.open(path_exploration);  

			myfile << "Iteration, Processing_Time, Distance, Area, LoopClosures, Current Error \n";
		}



/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		

		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map){
			map_info = map->info;		
			///////////////////////Occupancy to image	
			cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
			img.data = (unsigned char *)(&(map->data[0]) );
			cv::flip(img,img,0);
			image_map = img.clone();
			map_received = true;			
		}


/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
		}

////////////////
		void trajectoryCallback(const geometry_msgs::PoseArray &msg)
		{

			geometry_msgs::Point New_node = msg.poses.front().position;
			
			float difference_x = New_node.x - Last_node.x;
			float difference_y = New_node.y - Last_node.y;
			float delta_distance =	sqrt( difference_x*difference_x +  difference_y*difference_y);
			
			distance += delta_distance;
			// UNSTUCK
			{

				
				if( delta_distance  < 0.05){
					counter++;
		//			std::cout << "Counter is " << counter << std::endl;
				}
				else{
					counter=0;
				}
				
				if(counter > 20){
					geometry_msgs::PoseStamped pose_out;
					pose_out.pose.orientation = msg.poses.front().orientation;
					double angle = 2*atan2(pose_out.pose.orientation.z, pose_out.pose.orientation.w);
	
					float dist = 0.15;
					pose_out.pose.position.x = New_node.x + dist*cos(angle);
					pose_out.pose.position.y = New_node.y + dist*sin(angle);
					
					publish_goal(pose_out);		
					
					//Reset last goal

					Last_goal.header.seq=-1;
					//*/		
				}
			}
			Last_node = New_node;
			path_received = true;
		}

///////////////////////
		void ground_truth_Callback(const visualization_msgs::Marker& GT_msg){
			ground_truth= GT_msg.points;
			GT_received= true;
		}


////////////////
		void graphCallback(const visualization_msgs::Marker& graph_msg)
		{
			edges = graph_msg.points;
			graph_received = true;
		}

///////////////
		void UncertaintyCallback(const geometry_msgs::PoseWithCovariance& msg)
		{
			int a=1;
		}
//////////////
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			std::cout << "New Image"<< std::endl;
			cv_bridge::CvImagePtr temporal_ptr;

			try
			{
			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
//			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}
			
			image_tagged = temporal_ptr->image.clone();
			tagged_image_received = true;
			
			image_tagged.convertTo(image_tagged, CV_8UC1);

			bool data_ready = map_received & path_received & graph_received & tagged_image_received & GT_received;	
			cv::Mat grad;
			
			


			if(data_ready){
				cv::Mat occupancy_image = image_map.clone();
				


				RegionGraph Tremaux_Graph;

				clock_t time_before = clock();
				Tremaux_Graph.build_Region_Graph(edges, map_info, image_tagged, occupancy_image);
				clock_t time_after = clock();

				float time_elapsed = 1000*((float)(time_after - time_before) )/CLOCKS_PER_SEC;
//				std::cout << "Elapsed time in building: "<< time_elapsed <<" ms" << std::endl<< std::endl;
//				std::cout << Tremaux_Graph;			

//				time_before = clock();
				int region_completed = Tremaux_Graph.Tremaux_data(pose_to_publish) ;  
				time_after = clock();
				time_elapsed = 1000*((float)(time_after - time_before) )/CLOCKS_PER_SEC;
				{// RESULTS
					std::cout << std::endl<< std::endl;
					std::cout << "Current distance traveled: "<< distance <<" m" << std::endl;
									
					time_counter ++;				cum_time += time_elapsed;
					float Area = (map_info.resolution)*(map_info.resolution)* (cv::countNonZero(image_tagged) ) ;
	
					std::cout << "Elapsed time average: "<< cum_time/time_counter <<" ms" << std::endl;
	//				std::cout << "Number of Loop Closures: "<< Tremaux_Graph.number_loop_closures()  << std::endl;
					std::cout << "Area: "<< Area  << std::endl;
					

					
					float average_error;
					std::vector<float> error_list = Tremaux_Graph.extract_error_per_node(ground_truth, average_error);

					float current_error = 		error_list.back();			
					std::cout << "Current_error: "<< current_error  << std::endl;
					std::cout << "Average_error: "<< average_error  << std::endl;
					
					std::cout<< std::endl;
					
					
					myfile << time_counter <<", "<< time_elapsed <<", "<<  distance <<", "<< Area <<", "<<  Tremaux_Graph.number_loop_closures()<<", "<< current_error <<"\n";
				}


				//*
				if (region_completed < 0  & Last_goal.header.seq != -1){
//					Tremaux_Graph.connect_inside_region(pose_to_publish);

					float difference_x = Last_goal.pose.position.x - Last_node.x;
					float difference_y = Last_goal.pose.position.y - Last_node.y;
					
					float diff = sqrt( difference_x*difference_x +  difference_y*difference_y);
					Last_goal.header.seq = 0;
						
					if(diff < 0.1){				// Is it near the pose?
						publish_goal(pose_to_publish);
						Last_goal.pose = pose_to_publish.pose;
						Last_goal.header.seq = -1;
					}
					else{
						std::cout << " Trying to connect Sub-Graphs" << std::endl;		
					}
					
				}
				else
				//*/
				{
					publish_goal(pose_to_publish);
					Last_goal.pose = pose_to_publish.pose;
					Last_goal.header.seq = 0;
				}	
				publish_markers(Tremaux_Graph.collect_all_frontiers());
				
				cv::Mat edge_image = Tremaux_Graph.segment_current_frontier ( image_tagged );
				grad = edge_image.clone();

				
				data_ready = map_received = path_received = graph_received = tagged_image_received = GT_received = false;
//				ground_truth.push_back(current_ground_truth);
				std::cout<<"Ground Truth size "<< ground_truth.size() <<std::endl;
				std::cout << std::endl << std::endl;	
				
			}
			else{
//				grad = image_tagged;
			}
			
//			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			grad.convertTo(grad, CV_32F);
			cv_ptr->encoding = cv_ptr->encoding = "bgr8";			grad.convertTo(grad, CV_32F);
			grad.copyTo(cv_ptr->image);////most important
					
		}


////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
		}

		
		int publish_markers(std::vector < std::complex <double> >  region_frontier) {
			
			visualization_msgs::Marker marker;
		
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();

			
		
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::POINTS;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0;
			marker.pose.position.y = 0;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		
			
		
			marker.points.clear();

			for(int i=0;i< region_frontier.size();i++){
				geometry_msgs::Point point_from;
				
				point_from.x = region_frontier[i].real();
				point_from.y = region_frontier[i].imag();
				point_from.z = 0;
				
				marker.points.push_back(point_from);
			}
			
			
		
		
			markers_pub_.publish( marker );	
			
			
			return 0;
		}
		
		int publish_goal(geometry_msgs::PoseStamped  goal_msg) {
			goal_msg.header.frame_id = "map";
			goal_msg.header.stamp = ros::Time();
			
			goal_pub_.publish(goal_msg);
		}

/////////////////////////
//// UTILITY
/////////////////////////


		typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;





};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Exploration_Tremaux");
	
	std::string mapname = "map";	
	ROS_handler mg(mapname);
	ros::spin();
	
	return 0;
}
