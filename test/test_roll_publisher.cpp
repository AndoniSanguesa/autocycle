#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_roll_pub");
  ros::NodeHandle nh;

  ros::Publisher roll_pub = nh.advertise<std_msgs::Float64>("sensors/roll", 1);

  float roll = 0;

  srand(time(NULL));

  while(ros::ok()){
    roll = ((double) rand() / (RAND_MAX)) / 2;
    std_msgs::Float64 flt;
    flt.data = roll;
    roll_pub.publish(flt);
  }
}
