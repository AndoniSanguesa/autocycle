#include <ros/ros.h>
#include <autocycle/GetData.h>
#include <autocycle/RollAdj.h>
#include <autocycle/Point.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <math.h>

int PI = 3.14159265;

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_roll_adj");
  ros::NodeHandle nh;

  ros::service::waitForService("fix_roll");
  ros::service::waitForService("get_data");

  ros::ServiceClient get_data_cli = nh.serviceClient<autocycle::GetData>("get_data");
  ros::ServiceClient fix_roll_cli = nh.serviceClient<autocycle::RollAdj>("fix_roll");

  autocycle::RollAdj::Request fix_roll_req;
  autocycle::RollAdj::Response fix_roll_resp;

  autocycle::GetData::Request get_data_req;
  autocycle::GetData::Response get_data_resp;

  srand(time(NULL));

  int i = 0;
  float res_x = 0;
  float res_y = 0;
  float res_z = 0;
  float exp_x = 0;
  float exp_y = 0;
  float roll = 0;
  vector<autocycle::Point> vec;
  autocycle::Point p;
  for(i=0;i<100;i++){
    vec.clear();
    p.x = rand() % 200 - 100;
    p.y = rand() % 200 - 100;
    p.z = rand() % 200 - 100;
    vec.push_back(p);
    fix_roll_req.in = vec;
    get_data_req.data_type = "roll";
    get_data_cli.call(get_data_req, get_data_resp);
    roll = get_data_resp.data;
    fix_roll_req.roll = roll;
    fix_roll_cli.call(fix_roll_req, fix_roll_resp);
    res_x = round(fix_roll_resp.out[0].x * 1000.0) / 1000.0;
    res_y = round(fix_roll_resp.out[0].y * 1000.0) / 1000.0;
    res_z = round(fix_roll_resp.out[0].z * 1000.0) / 1000.0;
    exp_x = round(((p.x*cos(roll))-(p.y*sin(roll))) * 1000.0) / 1000.0;
    exp_y = round(((p.x*sin(roll))+(p.y*cos(roll))) * 1000.0) / 1000.0;
    if(!(res_x == exp_x && res_y == exp_y && res_z == p.z)){
      ROS_WARN_STREAM("TEST FAILED: \n  ROLL: " << roll << " -- ORIGINAL POINT: (" << p.x << ", " << p.y << ", " << p.z << ") -- ADJUSTED POINT: (" << res_x << ", " << res_y << ", " << res_z << ")" << " -- EXPECTED POINT: (" << exp_x << ", " << exp_y << ", " << p.z << ")");
      return 0;
    }
  }
  ROS_INFO_STREAM("ALL TESTS FOR ROLL ADJUSTMENT PASSED");
  ros::shutdown();
}
