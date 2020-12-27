// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/Lidar.h>
#include <autocycle/LvxData.h>
#include <std_msgs/String.h>


// TEMPORARY CALLBACK FUNCTION TO TEST THE PARSER
void TestFrames(const autocycle::LvxData &msg) {
  for(int i=0; i < 5; i++){
    ROS_INFO_STREAM("x value at" << i <<  ": " << msg.xs.at(i));
  }
  ROS_INFO_STREAM("LVX file analyzed");
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
  }
}
