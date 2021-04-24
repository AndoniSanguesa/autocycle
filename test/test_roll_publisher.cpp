#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  // Registers node with the master
  ros::init(argc, argv, "test_roll_pub");
  ros::NodeHandle nh;

  // Creates publisher that will publish roll values
  ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("sensors/roll", 1);

  // Initializes roll value
  float roll = 0;

  // Sets seed for rand
  srand(time(NULL));

  // Continuously publishes roll values
  while(ros::ok()){
    // Generates random roll value
    roll = ((double) rand() / (RAND_MAX)) / 2;

    // Publishes that roll value
    std_msgs::Float64 flt;
    flt.data = roll;
    roll_pub.publish(flt);
  }
}
