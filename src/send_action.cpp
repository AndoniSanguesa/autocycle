#include <ros/ros.h>
#include <serial/serial.h>
#include <autocycle_extras/GetDelta.h>
#include <autocycle_extras/CalcDeltas.h>
#include <autocycle_extras/Data.h>
#include <std_msgs/Float32.h>
#include <chrono>
#include <std_srvs/Empty.h>
#include <unistd.h>

using namespace std;

// Creates serial object to write to
serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

float dist_trav = 0;
float velocity = 0;
float state = 0;
float time_elapsed = 0;

void reset_distance(const autocycle_extras::CalcDeltas data){
    dist_trav = 0;
}

void update_velocity(const autocycle_extras::Data data){
    velocity = data.data[5];
}

void update_state(const std_msgs::Float32 data){
    state = data.data;
}

int main(int argc, char **argv) {
    // Registers node with the master.
    ros::init(argc, argv, "read_serial");
    ros::NodeHandle nh;

    ros::service::waitForService("due_ready");
    ros::service::waitForService("get_delta");

    // Creates service proxy that will grab new deltas
    ros::ServiceClient get_delta = nh.serviceClient<autocycle_extras::GetDelta>("get_delta");
    autocycle_extras::GetDelta::Request req;
    autocycle_extras::GetDelta::Response resp;

    // Initializes delta requests and response. Used to wait for an actual delta
    resp.delta = -1;
    req.vel = 0;
    req.x = 0;

    // Creates Service that will reset the distance
    ros::Subscriber get_path = nh.subscribe("cycle/calc_deltas", 1, &reset_distance);

    // Creates subscriber for updating velocity
    ros::Subscriber get_vel = nh.subscribe("sensors/data", 1, &update_velocity);

    // Creates subscriber for updating state
    ros::Subscriber get_state = nh.subscribe("sensors/state", 1, &update_state);

    // Starts the clock!
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
    // Waits until a path is ready to be followed
    while(resp.delta == -1){
        get_delta.call(req, resp);
    }

    // Rates the delta querying speed
    // ros::Rate loop_rate(10);

    my_serial.write("t1,10000;");

    while(ros::ok()){
	// loop_rate.sleep();
        ros::spinOnce();

        if(state!=0 && state!=3){
            ROS_WARN_STREAM("state: " << state);
	    ros::shutdown();
        }

        end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        start = std::chrono::high_resolution_clock::now();
        dist_trav += velocity * (((float) duration.count())/1000.0);
        req.x = dist_trav;
        req.vel = velocity;
        get_delta.call(req, resp);

        usleep(250000);

        if(resp.delta < 0.25 && resp.delta > -0.25){
            my_serial.write("d" + to_string(resp.delta) + ";");
        }
    }
    my_serial.write("s 0;");
}
