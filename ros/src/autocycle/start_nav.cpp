// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/Lidar.h>
#include <autocycle/LvxData.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>
#include <std_msgs/String.h>


// TEMPORARY CALLBACK FUNCTION TO TEST THE PARSER
void TestFrames(const autocycle::LvxData &msg) {
  ROS_WARN_STREAM("THIS ANALYSIS IS TEMPORARY AND FOR TESTING PURPOSES ONLY. SHOULD BE REPLACED BY OBJECT GENERATING ALGOITHM");
  for(int i=0; i < 5; i++){
    ROS_INFO_STREAM("x value at" << i <<  ": " << msg.xs.at(i));
  }
  ROS_INFO_STREAM("LVX file analyzed");
}

// TEMPORARY CALLBACK FUNCTION TO TEST THE PARAMETRIC FUNCTIONS
void TestParam(const std_msgs::String &msg) {
  ROS_WARN_STREAM("THIS INFORMATION IS ONLY TEMPORARILY ACCESSED HEAR UNTIL I N T E G R A T I O N");
  ROS_INFO_STREAM("Parametric fx: " << msg.data);
}

// Time in seconds to run LiDAR at each call
int lidar_time = 5;

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the run_lidar service to be active
  ros::service::waitForService("run_lidar");

  // Register a server client with the master.
  // Responsible for calling on the run_lidar service
  ros::ServiceClient lidar_client = nh.serviceClient<autocycle::Lidar>("run_lidar");

  // TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
  // ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
  ros::Subscriber lvx_sub = nh.subscribe("LiDAR/data", 3, &TestFrames);

  // TEMPORARY: publisher responsible for generating dummy object data
  ros::Publisher object_pub = nh.advertise<autocycle::ObjectList>("bezier/objects", 3);

  // PROBABLY TEMPRORARY: subscription to bezier/param. Used to confirm output
  ros::Subscriber path_sub = nh.subscribe("bezier/param", 1, &TestParam);

  // Registers a Publisher with the master.
  // Responsible for publishing the path of
  // lvx files to the LiDAR/path topic
  ros::Publisher lvx_pub = nh.advertise<std_msgs::String>("LiDAR/path", 1);

  // The response and request objects that will contain data regarding lidar
  autocycle::Lidar::Request req;
  autocycle::Lidar::Response resp;

  // Assign the appropriate data to the request object
  req.time = lidar_time;

  // Navigation loop
  while(ros::ok()){
    ROS_INFO_STREAM("Sending request for LVX file.");

    // Calls on the run_lidar node to record data
    bool result = lidar_client.call(req, resp);

    if(!result){
      ROS_FATAL_STREAM("LVX FILE COULD NOT BE CREATED");
      exit(1);
    }

    ROS_INFO_STREAM("LVX file generated.");
    ROS_INFO_STREAM("Sending request to analyze LVX File.");

    // Publishes the determined lvx file to LiDAR/path
    std_msgs::String msg;
    msg.data = resp.path;
    lvx_pub.publish(msg);

    // Checks Subscriptions
    ros::spinOnce();

    ROS_INFO_STREAM("Sending Object data to path planning");
    ROS_WARN_STREAM("THIS IS TEMPORARY UNTIL WE CAN GET OBJECT DATA");
    // TEMPORARY: Sends dummy object data to bezier/objects to test for now
    autocycle::Object o1 ;
    autocycle::ObjectList ol1;
    o1.edge_to_path = 2;
    o1.dist_to_edge = 3;
    o1.edge_len = 3;
    ol1.obj_lst.push_back(o1);
    object_pub.publish(ol1);
  }
}
