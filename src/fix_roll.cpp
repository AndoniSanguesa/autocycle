#include <ros/ros.h>
#include <autocycle/Point.h>
#include <autocycle/RollAdj.h>
#include <cmath>
#include <vector>

using namespace std;

int PI = 3.14159265;

bool fixRoll(
    autocycle::RollAdj::Request &req,
    autocycle::RollAdj::Response &resp
) {
  // Initializes variables
  int i = 0;
  vector<autocycle::Point> ret;
  autocycle::Point cur_point;
  float deg;


  ROS_INFO_STREAM("ROLL RECEIVED WAS: " << req.roll);

  // Updates each point in `req` and pushes it to the new vector
  for(i=0;i<req.in.size();i++){
    autocycle::Point new_point;
    cur_point = req.in[i];
    new_point.x = (cur_point.x*cos(req.roll)) - (cur_point.y*sin(req.roll));
    new_point.y = (cur_point.x*sin(req.roll)) + (cur_point.y*cos(req.roll));
    new_point.z = cur_point.z;
    ret.push_back(new_point);
  }

  // Updates the response value to be the new vector
  resp.out = ret;
  return true;
}

int main(int argc, char **argv) {
  // Initialize the node and register it with the master
  ros::init(argc, argv, "fix_roll");
  ros::NodeHandle nh;

  // Register Service Server with the master.
  ros::ServiceServer fix_roll_server = nh.advertiseService("fix_roll", &fixRoll);

  // Constantly checks for new data to process
  ros::spin();
}
