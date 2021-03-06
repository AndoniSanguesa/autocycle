#include <ros/ros.h>
#include <serial/serial.h>
#include <autocycle_extras/GetDelta.h>
#include <std_msgs/Float32.h>
#include <chrono>
#include <std_srvs/Empty.h>

using namespace std;

float dist_trav = 0;
float velocity = 0;
float roll = 0;
float time_elapsed = 0;

bool reset_distance(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp
){
    dist_trav = 0;
    return true;
}

void update_velocity(const std_msgs::Float32 data){
    velocity = data.data;
}

void update_roll(const std_msgs::Float32 data){
    roll = data.data;
}

int main(int argc, char **argv) {
    // Registers node with the master.
    ros::init(argc, argv, "read_serial");
    ros::NodeHandle nh;

    // Creates service proxy that will grab new deltas
    ros::ServiceClient get_delta = nh.serviceClient<autocycle_extras::GetDelta>("get_delta");
    autocycle_extras::GetDelta::Request req;
    autocycle_extras::GetDelta::Response resp;

    // Creates Service that will reset the distance
    ros::ServiceServer get_path = nh.advertiseService("reset_action", &reset_distance);

    // Creates subscriber for updating velocity
    ros::Subscriber get_vel = nh.subscribe("sensors/vel", 1, &update_velocity);

    // Creates subscriber for updating roll
    ros::Subscriber get_roll = nh.subscribe("sensors/roll", 1, &update_roll);

    // Starts the clock!
    auto start = std::chrono::high_resolution_clock::now();

    while(ros::ok()){
        ros::spinOnce();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        auto start = std::chrono::high_resolution_clock::now();
        dist_trav += velocity * (((float) duration.count())/1000.0);
        req.x = dist_trav;
        req.roll = roll;
        get_delta.call(req, resp);
    }
}
