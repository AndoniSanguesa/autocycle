#include <ros/ros.h>
#include <autocycle/Heading.h>
#include <std_msgs/Float64.h>

// Will contain the new roll value!
float heading = -1;

void getHeadingFromTopic(const std_msgs::Float64 new_heading){
  heading = new_heading.data;
}

bool getHeading(
    autocycle::Heading::Request &req,
    autocycle::Heading::Response &resp
){
  ros::NodeHandle nh;

  // Creates subscriber for the roll data, Collects data, deletes subscriber
  ros::Subscriber roll_sub = nh.subscribe("sensors/heading", 1, &getHeadingFromTopic);
  while(heading == -1){
    ros::spinOnce();
  }
  roll_sub.shutdown();

  // Stores the roll in the response variable
  resp.heading = heading;
  heading = -1;
  return true;
}


int main(int argc, char **argv){
  //Initaialize the node and register it wth the master.
  ros::init(argc, argv, "get_heading");
  ros::NodeHandle nh;

  // Register Service Server with the master.
  ros::ServiceServer get_heading_server = nh.advertiseService("get_heading", &getHeading);

  // Constantly checks for requests
  ros::spin();
}
