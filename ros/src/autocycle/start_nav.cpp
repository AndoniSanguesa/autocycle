// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/Lidar.h>

// Time in seconds to run LiDAR at each call
int lidar_time = 5;

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the run_lidar service to be active
  ros::service::waitForService("run_lidar");

  // Register a server client with the master.
  ros::ServiceClient lidar_client = nh.serviceClient<autocycle::Lidar>("run_lidar");

  // The response and request objects that will contain data regarding lidar
  autocycle::Lidar::Request req;
  autocycle::Lidar::Response resp;

  // Assign the appropriate data to the request object
  req.time = lidar_time;

  // Navigation loop
  while(ros::ok()){
    // Calls on the run_lidar node to record data
    lidar_client.call(req, resp);
  }
}
