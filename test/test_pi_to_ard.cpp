#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

// Creates serial object to write to
serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

int main(int argc, char **argv){
    ros::init(argc, argv, "test_pi_to_ard");
    ros::NodeHandle nh;

    if(argc == 1){
        std::cout << "Please specify an angle in degrees (e.g. `roslaunch test_pi_to_ard.launch ang:='10'" << std::endl;
        return 1;
    }

    int deg = std::stoi(argv[1]);

    if(deg > 30 || deg < -30){
        std::cout << "That's tooo much man" << std::endl;
        return 1;
    } 

    while(ros::ok()){
        my_serial.write("d " + std::to_string(deg * 0.01745329251) + ";");
    }
}
