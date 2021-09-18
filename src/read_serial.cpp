#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

int main(int argc, char **argv){
  // Registers node with the master.
  ros::init(argc, argv, "read_serial");
  ros::NodeHandle nh;

  // Registers the appropriate publishers
  ros::Publisher state_pub = nh.advertise<std_msgs::Float32>("sensors/state", 1);
  ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("sensors/roll", 1);
  ros::Publisher steer_pub = nh.advertise<std_msgs::Float32>("sensors/steer", 1);
  ros::Publisher droll_pub = nh.advertise<std_msgs::Float32>("sensors/droll", 1);
  ros::Publisher dsteer_pub = nh.advertise<std_msgs::Float32>("sensors/dsteer", 1);
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("sensors/vel", 1);
  ros::Publisher torque_pub = nh.advertise<std_msgs::Float32>("sensors/torque", 1);
  ros::Publisher head_pub = nh.advertise<std_msgs::Float32>("sensors/heading", 1);
  ros::Publisher dhead_pub = nh.advertise<std_msgs::Float32>("sensors/dheading", 1);
  ros::Publisher met_pub = nh.advertise<std_msgs::Float32>("sensors/met", 1);

  // Initializing vars
  string temp = "";
  char back_char;
  std_msgs::Float32 to_pub;
  int cur_publisher = 0;
  ros::Publisher publishers [10] = {state_pub, roll_pub, steer_pub, droll_pub, dsteer_pub, vel_pub, torque_pub, head_pub, dhead_pub, met_pub};

  // Creates serial object to read from
  // TODO: Figure out what to do with the timeout
  serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

  bool ready = false;
  while(!ready){
    temp.append(my_serial.read());
    
    if((char) temp.back() == '\n'){
      temp.pop_back();
      ROS_INFO_STREAM("READ LINE : " << temp);
      if(temp.find("setup") != -1){
         ROS_INFO_STREAM("I THINK THE LINES ARE THE SAME!!!!");
      ready = true;
      }
      temp.clear();
    }
  }
  
  while(ros::ok()){
    temp.append(my_serial.read());
    back_char = (char) temp.back();
    if(back_char == '\t' || back_char == '\n'){
      temp.pop_back();
      istringstream temp_as_stream(temp);
      temp_as_stream >> to_pub.data;
      temp.clear();
      if (cur_publisher%16 < 10){
        publishers[cur_publisher%16].publish(to_pub);
      }
      cur_publisher++;
    }
  }
}
