// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/Lidar.h>
#include <autocycle/LvxData.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>
#include <std_msgs/String.h>

// Time in seconds to run LiDAR at each call
int lidar_time = 5;

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the run_lidar service to be active
  ros::service::waitForService("run_lidar");

  // Wait for the parse_lvx service to be active
  ros::service::waitForService("parse_lvx");

  // Wait for the plan_path service to be active
  ros::service::waitForService("plan_path");

  // Register a server client with the master.
  // Responsible for calling on the run_lidar service
  ros::ServiceClient lidar_client = nh.serviceClient<autocycle::Lidar>("run_lidar");

  // TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
  // ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
  ros::ServiceClient lvx_client = nh.serviceClient<autocycle::LvxData>("parse_lvx");

  //TEMPORARY UNTIL I N T E G R A T I O N
  ros::ServiceClient path_client = nh.serviceClient<autocycle::ObjectList>("plan_path");

  // The response and request objects that will contain data regarding lidar
  autocycle::Lidar::Request lidar_req;
  autocycle::Lidar::Response lidar_resp;

  // The response and request objects that will contain data regarding the lvx file
  autocycle::LvxData::Request lvx_req;
  autocycle::LvxData::Response lvx_resp;

  // The response and request objects that will contain data regarding the path
  autocycle::ObjectList::Request path_req;
  autocycle::ObjectList::Response path_resp;

  // Assign the appropriate data to the request object
  lidar_req.time = lidar_time;

  // Navigation loop
  while(ros::ok()){
    ROS_INFO_STREAM("Sending request for LVX file.");

    // Calls on the run_lidar node to record data
    bool result = lidar_client.call(lidar_req, lidar_resp);

    if(!result){
      ROS_FATAL_STREAM("LVX FILE COULD NOT BE CREATED");
      exit(1);
    }

    ROS_INFO_STREAM("LVX file generated.");
    ROS_INFO_STREAM("Sending request to analyze LVX File.");

    // Publishes the determined lvx file to LiDAR/path
    lvx_req.path = lidar_resp.path;
    result = lvx_client.call(lvx_req, lvx_resp);

    if(!result){
        exit(1);
    }

    // TEMPORARY: analyzes results of the parse_lvx fe
    ROS_WARN_STREAM("TEMPORARY LVX DATA ANALYSIS FOR TESTING UNTIL JACOBS ALGORITHM IS DONE");
    for(int i = 0; i < 5; i++){
        ROS_INFO_STREAM("x value at " << i << " : " << lvx_resp.xs.at(i));
    }
    ROS_INFO_STREAM("LVX file analyzed.");

    ROS_INFO_STREAM("Sending Object data to path planning");
    ROS_WARN_STREAM("THIS IS TEMPORARY UNTIL WE CAN GET OBJECT DATA");

    // TEMPORARY: Sends dummy object data to bezier/objects to test for now
    autocycle::Object o1 ;
    autocycle::ObjectList ol1;
    o1.edge_to_path = 2;
    o1.dist_to_edge = 3;
    o1.edge_len = 3;
    path_req.obj_lst.push_back(o1);

    // Report parameterized equation for the calculated path
    path_client.call(path_req, path_resp);
    ROS_WARN_STREAM("THIS INFORMATION SHOULD ONLY BE REPORTED HERE UNTIL I N T E G R A T I O N");
    ROS_INFO_STREAM("Param for path: " << path_resp.param);
  }
}
