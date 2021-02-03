#include <ros/ros.h>
#include <autocycle/Roll.h>
#include <std_msgs/Float64.h>

// Will contain the new roll value!
float roll;

// Allows for the creation of services and subscribers
ros::NodeHandle nh;

void getRollFromTopic(const std_msgs::Float64 new_roll){
  roll = new_roll.data;
}

bool getRoll(
    autocycle::Roll::Request &req,
    autocycle::Roll::Response &resp
){
  // Creates subscriber for the roll data, Collects data, deletes subscriber
  ros::Subscriber roll_sub = nh.subscribe("sensors/roll", 1, &getRollFromTopic);
  ros::spinOnce();
  roll_sub.shutdown();

  // Stores the roll in the response variable
  resp.roll = roll;
  return true;
}


int main(int argc, char **argv){
  //Initaialize the node and register it wth the master.
  ros::init(argc, argv, "get_roll");

  // Register Service Server with the master.
  ros::ServiceServer get_roll_server = nh.advertiseService("get_roll", &getRoll);

  // Constantly checks for requests
  ros::spin();
}
