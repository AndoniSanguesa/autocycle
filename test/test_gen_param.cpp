#include <ros/ros.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>
#include <autocycle/Curve.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

using namespace std;

string param = "";

void GetParam(autocycle::Curve msg){
    param = msg.param;

    // Prints the discovered curve
    ROS_INFO_STREAM("PARAM CURVE: " << param);
    ros::shutdown();
}

int main(int argc, char **argv){

  // Registers node with the master
  ros::init(argc, argv, "test_gen_param");
  ros::NodeHandle nh;

  ros::Publisher obj_pub = nh.advertise<autocycle::ObjectList>("cycle/ObjectFrame", 1);

  // Subscriber that will be listening for a new curve
  ros::Subscriber param_sub = nh.subscribe("cycle/param", 1, &GetParam);

  // Sets the rand seed
  srand(time(NULL));

  // The number of objects to be generated (random number between 1 and 5)
  int num_obj = rand() % 5 + 1;

  // Generates the objects, adds them to the request and prints out their properties
  for(int i = 0; i < num_obj; i++){
    autocycle::Object o;
    autocycle::ObjectList ol;
    o.x1 = (rand() % 10) + ((double) rand() / (RAND_MAX));
    o.x2 = (rand() % 10) + ((double) rand() / (RAND_MAX));
    o.y1 = (rand() % 10) + ((double) rand() / (RAND_MAX));
    o.y2 = (rand() % 10) + ((double) rand() / (RAND_MAX));
    ol.push_back(o);
    obj_pub.publish(ol);
    ROS_INFO_STREAM("OBJECT " << i << "--> (" << o.x1 << ", " << o.y1 << "), (" << o.x2 << ", " << o.y2 << ")");
  }

  // Calls on path planning to do its work and listens for its response
  ros::spin();
}
