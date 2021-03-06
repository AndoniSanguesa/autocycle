#include <ros/ros.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>

int main(int argc, char **argv){
    // Initializes the node with the master
    ros::init(argc, argv, "test_bezier");
    ros::NodeHandle nh;

    // Creates Publisher that will publish the object list
    ros::Publisher obj_pub = nh.advertise<autocycle_extras::ObjectList>("cycle/object_frame", 1);

    // Creates Object and publishes it
    autocycle_extras::ObjectList ol;
    autocycle_extras::Object o;
    o.x1 = 2000;
    o.x2 = -1000;
    o.z1 = 5000;
    o.z2 = 7000;
    ol.obj_lst.push_back(o);
    ROS_INFO_STREAM(ol);
    while(ros::ok()){
        obj_pub.publish(ol);
    }
}
