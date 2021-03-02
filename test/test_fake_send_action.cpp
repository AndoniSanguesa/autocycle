#include <ros/ros.h>
#include <autocycle/Action.h>

bool SendAction(
        autocycle::Action::Request &req,
        autocycle::Action::Response &resp

){
    
    return true;
}

int main(int argc, char **argv){
    // Registers node with the master
    ros::init(argc, argv, "test_fake_send_action");
    ros::NodeHandle nh;

    // Registers fake Service Server with master.
    ros::ServiceServer act_srv = nh.advertiseService("send_action", &SendAction);

    ros::spin();
}
