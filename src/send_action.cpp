#include <ros/ros.h>
#include <serial/serial.h>
#include <autocycle_extras/GetDelta.h>
#include <std_msgs/Float32.h>
#include <chrono>

using namespace std;

// Creates serial object to write to
serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

float distance = 0;
float velocity = 0;
float roll = 0;
float time_elapsed = 0;

void reset_distance(const std_msgs::Empty e){
    distance = 0;
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
    std::ServiceClient get_delta = nh.serviceClient<autocycle_extras::GetDelta>("get_delta");
    std::autocycle_extras::GetDelta::Request req;
    std::autocycle_extras::GetDelta::Response resp;

    // Creates subscriber for new Path
    std::Subscriber get_path = nh.subscribe("cycle/new_path", 1, &reset_distance);

    // Creates subscriber for updating velocity
    std::Subscriber get_vel = nh.subscribe("sensors/vel", 1, &update_velocity)

    // Creates subscriber for updating roll
    std::Subscriber get_roll = nh.subscribe("sensors/roll", 1, &update_velocity)

    // Starts the clock!
    auto start = std::chrono::high_resolution_clock::now();

    while(ros::ok()){
        ros::spinOnce();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        auto start = std::chrono::high_resolution_clock::now();
        distance += velocity * (((float) duration.count())/1000.0)
        req.x = distance;
        req.roll = roll;
        get_delta(req. resp);

        my_serial.write("s 4.5;d " + to_string(resp.delta) + ";");
    }

}
