#include <ros/ros.h>
#include <autocycle_extras/GetData.h>
#include <std_msgs/Float32.h>

using namespace std;

float vel = 0;
float head = 0;
float roll = 0;
float met = 0;

void get_vel(const std_msgs::Float32 data){
    vel = data.data;
}

void get_head(const std_msgs::Float32 data){
    head = data.data;
}

void get_roll(const std_msgs::Float32 data){
    roll = data.data;
}

void get_met(const std_msgs::Float32 data){
    met = data.data;
}

bool get_data(
    autocycle_extras::GetData::Request &req,
    autocycle_extras::GetData::Response &resp
) {
  switch(req.data_type){
    case(0):
        resp.data = vel;
	break;
    case(1):
        resp.data = head;
	break;
    case(2):
        resp.data = roll;
	break;
    case(3):
        resp.data = met;
	break;
  }

  return true;
}

int main(int argc, char **argv){
  // Registers node with the master.
  ros::init(argc, argv, "get_data");
  ros::NodeHandle nh;

  // Register Service Server with the master
  ros::ServiceServer get_data_serv = nh.advertiseService("get_data", &get_data);

  ros::Subscriber vel_sub = nh.subscribe("sensors/vel", 1, &get_vel);
  ros::Subscriber head_sub = nh.subscribe("sensors/heading", 1, &get_head);
  ros::Subscriber roll_sub = nh.subscribe("sensors/roll", 1, &get_roll);
  ros::Subscriber met_sub = nh.subscribe("sensors/met", 1, &get_met);

  // Constantly checks for data requests
  ros::spin();
}
