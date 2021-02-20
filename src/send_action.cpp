#include <ros/ros.h>
#include <autocycle/Action.h>
#include <serial/serial.h>

using namespace std;

// Creates serial object to write to
serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

bool SendAction(
        autocycle::Action::Request &req,
        autocycle::Action::Response &resp

){
    string to_write = "";

    if(req.coms[0]){
        to_write.append("s " + to_string(req.speed) + ";");
    }
    if(req.coms[1]){
        to_write.append("d " + to_string(req.delta) + ";");
    }
    if(req.coms[2]){
        to_write.append("c " + req.command + ";");
    }

    my_serial.write(to_write);
    return true;
}

int main(int argc, char **argv) {
    // Registers node with the master.
    ros::init(argc, argv, "read_serial");
    ros::NodeHandle nh;

    // Registers Service Server with master.
    ros::ServiceServer act_srv = nh.advertiseService("send_action", &SendAction);

    // Waits for actions to send
    ros::spin();

}
