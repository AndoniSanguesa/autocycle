#include <ros/ros.h>
#include <serial/serial.h>
#include <autocycle_extras/GetDelta.h>
#include <autocycle_extras/CalcDeltas.h>
#include <std_msgs/Float32.h>
#include <chrono>
#include <std_srvs/Empty.h>

using namespace std;

// Creates serial object to write to
serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

float dist_trav = 0;
float velocity = 0;
float roll = 0;
float time_elapsed = 0;

void reset_distance(const autocycle_extras::CalcDeltas data){
    dist_trav = dist_trav;
}

void update_velocity(const std_msgs::Float32 data){
    velocity = data.data;
}

void update_roll(const std_msgs::Float32 data){
    roll = data.data;
}

int main(int argc, char **argv) {
    // Registers node with the master.
    ros::init(argc, argv, "test_send_only_ang");
    ros::NodeHandle nh;

    // Creates service proxy that will grab new deltas
    ros::ServiceClient get_delta = nh.serviceClient<autocycle_extras::GetDelta>("get_delta");
    autocycle_extras::GetDelta::Request req;
    autocycle_extras::GetDelta::Response resp;

    req.x = 0;
    req.vel = 0;
    resp.delta = -1;
	
    ros::service::waitForService("get_delta");

    // Creates Service that will reset the distance
    ros::Subscriber get_path = nh.subscribe("cycle/path", 1, &reset_distance);

    // Creates subscriber for updating velocity
    ros::Subscriber get_vel = nh.subscribe("sensors/vel", 1, &update_velocity);

    // Creates subscriber for updating roll
    ros::Subscriber get_roll = nh.subscribe("sensors/roll", 1, &update_roll);

    // Starts the clock!
    auto start = std::chrono::high_resolution_clock::now();

    // Waits until a path is ready to be followed
    while(resp.delta == -1){
        get_delta.call(req, resp);
    }

    // Rates the delta querying speed
    ros::Rate loop_rate(10);

    while(ros::ok()){
	    loop_rate.sleep();
        ros::spinOnce();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        auto start = std::chrono::high_resolution_clock::now();
        dist_trav += velocity * (((float) duration.count())/1000.0);
        //dist_trav += 0.25;
        req.x = dist_trav;
        req.vel = velocity;
        get_delta.call(req, resp);
        my_serial.write("s 1;");
        ROS_INFO_STREAM("DELTA OUTPUT: " << resp.delta);
        usleep(250000);
	    if(dist_trav >= 15){
	        return 1;
	    }
        my_serial.write("d " + to_string(resp.delta) + ";");
    }
}
