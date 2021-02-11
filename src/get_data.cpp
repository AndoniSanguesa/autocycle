#include <ros/ros.h>
#include <autocycle/GetData.h>
#include <std_msgs/Float64.h>

using namespace std;

string valid_data_types [9] = {"state", "roll", "steer", "droll", "dsteer", "vel", "heading", "dheading", "met"};

float data = -1;

void readTopic(const std_msgs::Float64 topic_data){
  data = topic_data.data;
}

bool get_data(
    autocycle::GetData::Request &req,
    autocycle::GetData::Response &resp
) {
  ros::NodeHandle nh;

  // Validates input
  if(find(begin(valid_data_types), end(valid_data_types), req.data_type) == end(valid_data_types)){
    ROS_ERROR_STREAM("COULD NOT GET DATA: '" << req.data_type << "'. Are you sure it exists?");
    return false;
  }

  // Creates subscriber for the data, collects that data and deletes subscriber
  ros::Subscriber data_sub = nh.subscribe("sensors/" + req.data_type, 1, &readTopic);
  while(data == -1){
    ros::spinOnce();
  }
  data_sub.shutdown();

  // Stores data in the response variable
  resp.data = data;
  data = -1;
  return true;
}

int main(int argc, char **argv){
  // Registers node with the master.
  ros::init(argc, argv, "get_data");
  ros::NodeHandle nh;

  // Register Service Server with the master
  ros::ServiceServer get_data_serv = nh.advertiseService("get_data", &get_data);

  // Constantly checks for data requests
  ros::spin();
}
