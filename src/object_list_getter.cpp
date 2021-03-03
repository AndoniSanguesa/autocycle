#include <ros/ros.h>
#include <autocycle/ObjectDetectionList.h>
#include <autocycle/ObjectList>

autocycle::ObjectList obj_lst;

bool comp_data(
    autocycle::ObjectDetectionList::Request &req,
    autocycle::ObjectDetectionList::Response &resp
){
    if(req.obj_lst != obj_lst.obj_lst){
        resp.obj_lst = obj_lst.obj_lst;
        return true;
    }
    return false;
}

void get_data(autocycle::ObjectList msg){
    obj_lst = msg.obj_lst;
}

int main(int argc, char **argv){
    // Registers Node with the Master
    ros::init(argc, argv, "object_list_getter");
    ros::NodeHandle nh;

    // Creates subscriber for Object Detection stuff
    ros::Subscriber obj_dect_sub = nh.Subscribe("cycle/objects", 1, &get_data)

    // Creates Service
    ros::ServiceServer serv = nh.advertiseService("object_list_getter", &comp_data);
    ros::spin();
}