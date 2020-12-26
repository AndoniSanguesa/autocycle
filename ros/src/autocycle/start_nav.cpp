// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <std_msgs/Int16.h>

// Time in seconds to run LiDAR at each call
int lidar_time = 5;

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Defines the publisher that will call on the LiDAR to operate.
  ros::Publisher lidar_pub = nh.advertize<std_msgs::Int16>("LiDAR/time_dur", 5);

  //Defines the subscriber that will wait for the LVX file.
  ros::Subscriber lidar_sub = nh.subscribe("LiDAR/path", 5);

  // Navigation loop
  while(ros::ok()){
    // Calls on the run_lidar node to record data
  }
}
