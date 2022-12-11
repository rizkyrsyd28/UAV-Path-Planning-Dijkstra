#include "../include/Implementation/UAV_Control.cpp"
// #include "../include/Implementation/ADT_Graph.cpp"
#include "../include/Implementation/ADT_Dijkstra.cpp"

const int alt = 1;

int main(int argc, char** argv) {
    	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node;
	
    std::vector<Coordinat> waypoint_img = Mapping();

	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(alt);

	std::vector <gnc_api_waypoint> waypointList;
	gnc_api_waypoint wp;
	for (int i = 0; i < waypoint_img.size(); i++){
		wp.x = waypoint_img[i].x * -0.025;
		wp.y = waypoint_img[i].y * 0.025;
		wp.z = alt;
		wp.psi = 5;
		waypointList.push_back(wp);
	}

	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;

}
