#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>
#include <fstream>
#include <sstream>

using namespace std;

int main(int argc, char **argv){
  // Registers node with the master.
  ros::init(argc, argv, "read_serial");
  ros::NodeHandle nh;

  // Registers the appropriate publishers
  ros::Publisher state_pub = nh.advertise<std_msgs::Float64>("sensors/state", 1);
  ros::Publisher roll_pub = nh.advertise<std_msgs::Float64>("sensors/roll", 1);
  ros::Publisher steer_pub = nh.advertise<std_msgs::Float64>("sensors/steer", 1);
  ros::Publisher droll_pub = nh.advertise<std_msgs::Float64>("sensors/droll", 1);
  ros::Publisher dsteer_pub = nh.advertise<std_msgs::Float64>("sensors/dsteer", 1);
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("sensors/vel", 1);
  ros::Publisher head_pub = nh.advertise<std_msgs::Float64>("sensors/heading", 1);
  ros::Publisher dhead_pub = nh.advertise<std_msgs::Float64>("sensors/dheading", 1);
  ros::Publisher met_pub = nh.advertise<std_msgs::Float64>("sensors/met", 1);

  // Initializing vars
  string temp = "";
  std_msgs::Float64 to_pub;
  int cur_publisher = 0;
  ros::Publisher publishers [9] = {state_pub, roll_pub, steer_pub, droll_pub, dsteer_pub, vel_pub, head_pub, dhead_pub, met_pub};

  // Creates serial object to read from
  // TODO: Figure out what to do with the timeout
  serial::Serial my_serial("/dev/ttyUSB0", (long) 0, serial::Timeout::simpleTimeout(0));

  while(ros::ok()){
    temp.append(my_serial.read());
    if((char) temp.back() == ','){
      temp.pop_back();
      istringstream temp_as_stream(temp);
      temp_as_stream >> to_pub.data;
      temp.clear();
      publishers[cur_publisher%9].publish(to_pub);
      cur_publisher++;
    }
  }
}
