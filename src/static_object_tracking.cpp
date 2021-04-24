#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <std_msgs/Float32.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>

float prev_heading = -1;
float heading = -1;
float velocity = -1;
std::vector<autocycle_extras::Object> ol;

void get_heading(const std_msgs::Float32 data){
    heading = data.data;
}

void get_velocity(const std_msgs::Float32 data){
    velocity = data.data;
}

void get_objects(const autocycle_extras::ObjectList data){
    ol.insert(ol.end(), data.obj_lst.begin(), data.obj_lst.end());
}

int main(int argc, char **argv){
    // Registers the Node with the Master
    ros::init(argc, argv, "static_object_tracking");
    ros::NodeHandle nh;

    // Creates subscriber that updates heading
    ros::Subscriber head_sub = nh.subscribe("sensors/heading", 1, &get_heading);

    // Creates subscriber that updates velocity
    ros::Subscriber vel_sub = nh.subscribe("sensors/vel", 1, &get_velocity);

    // Creates subscriber that gets latest object_list
    ros::Subscriber ol_sub = nh.subscribe("cycle/objects", 1, &get_objects);

    // Publishes objects to path topic
    ros::Publisher pub = nh.advertise<autocycle_extras::ObjectList>("cycle/object_frame", 1);

    // Initializing variables
    float delta_angle, delta_time, c, s, distance;
    std::vector<autocycle_extras::Object> new_objs;
    autocycle_extras::ObjectList to_pub;

    while (ol.size() == 0 || heading == -1 || velocity == -1){
        ros::spinOnce();
    }

    auto start = std::chrono::high_resolution_clock::now();
    while(ros::ok()){
        prev_heading = heading;
        ros::spinOnce();
        new_objs.clear();

        delta_angle = heading - prev_heading;

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        auto start = std::chrono::high_resolution_clock::now();
        delta_time = ((float) duration.count())/1000.0;

        c = cos(delta_angle);
        s = sin(delta_angle);
        distance = velocity * delta_time;

        for(auto & i : ol){
	    autocycle_extras::Object rotated;
            rotated.z1 = i.z1*c - i.x1*s;
            rotated.x1 = i.z1*s + i.x1*c - distance;
            rotated.z2 = i.z2*c - i.x2*s;
            rotated.x2 = i.z2*s + i.x2*c - distance;
            if(rotated.z1 < 0 && rotated.z2 < 0){
                continue;
            }
            new_objs.emplace_back(rotated);
        }
        ol.clear();
        ol = new_objs;
        to_pub.obj_lst = ol;
        pub.publish(to_pub);
	ol.clear(); // DELETE THIS LINE ONCE INTERSECTION IS WORKING
    }
}
