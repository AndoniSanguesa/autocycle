#include <ros/ros.h>
#include <autocycle/ObjectDetectionList.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>

autocycle::ObjectList obj_lst;

bool comp_data(
    autocycle::ObjectDetectionList::Request &req,
    autocycle::ObjectDetectionList::Response &resp
){
    if(req.obj_lst != obj_lst.obj_lst){
        resp.obj_lst = obj_lst.obj_lst;
    }
    return true;
}

void get_data(const autocycle::ObjectList msg){
    ROS_INFO_STREAM("BRUH at LISTENER");
    obj_lst = msg;
}

int main(int argc, char **argv){
    // Registers Node with the Master
    ros::init(argc, argv, "object_list_getter");
    ros::NodeHandle nh;

    // Creates subscriber for Object Detection stuff
    ros::Subscriber obj_dect_sub = nh.subscribe("cycle/objects", 1, &get_data);

    // Creates Service
    ros::ServiceServer serv = nh.advertiseService("object_list_getter", &comp_data);
    ros::spin();
}
