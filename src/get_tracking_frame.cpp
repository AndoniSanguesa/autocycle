#include <ros/ros.h>
#include <autocycle/ObjectList.h>

autocycle::ObjectList ol;
iden = -1;

bool get_tracking_frame(
    autocycle::ObjectList::Request &req,
    autocycle::ObjectList::Response &resp
){
    if(req.iden != iden){
        resp.iden = ol.iden;
        resp.obj_lst = ol.obj_lst;
    }else{
        resp.iden = -1;
    }
    return true;
}

void get_new_frame(const autocycle::ObjectList new_ol){
    ol = new_ol;
    iden = new_ol.iden;
}

int main(int argc, char **argv){
    // Registers the Node with the master
    ros::init(argc, argv, "get_tracking_frame");
    ros::NodeHandle nh;

    // Creates Subscriber that subscribes to the tracking frames
    ros::Subscriber track_sub = nh.subscribe("cycle/object_frame", 1, &get_new_frame);

    // Creates Service Server that will provide new tracking frames
    ros::ServiceServer track_serv = nh.advertiseServer("get_tracking_frame", &get_tracking_frame);

    // Runs service
    ros::spin();
}