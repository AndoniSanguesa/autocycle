#include <ros/ros.h>
#include <autocycle_extras/ObjectDetectionList.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>

autocycle_extras::ObjectList obj_lst;

bool comp_data(
    autocycle_extras::ObjectDetectionList::Request &req,
    autocycle_extras::ObjectDetectionList::Response &resp
){
    if(req.iden != obj_lst.iden){
        resp.obj_lst = obj_lst.obj_lst;
	resp.iden = obj_lst.iden;
    }
    return true;
}

void get_data(const autocycle_extras::ObjectList msg){
    ROS_INFO_STREAM("BRUH at LISTENER");
    obj_lst = msg;
}

int main(int argc, char **argv){
    // Registers Node with the Master
    ros::init(argc, argv, "object_list_getter");
    ros::NodeHandle nh;

    // Creates subscriber for Object Detection stuff
    ros::Subscriber obj_dect_sub = nh.subscribe("cycle/objects", 1, get_data);
    
    // Creates Service
    ros::ServiceServer serv = nh.advertiseService("object_list_getter", &comp_data);
    ros::spin();
}
