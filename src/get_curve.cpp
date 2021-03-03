#include <ros/ros.h>
#include <autocycle/GetCurve.h>
#include <autocycle/Curve.h>

autocycle::Curve curve;

bool comp_curves(
    autocycle::GetCurve::Request &req,
    autocycle::GetCurve::Response &resp
){
    if(req.param != curve.param){
        resp.param = curve.param;
        resp.length = curve.length;
    }
    return true;
}

void get_curve(const autocycle::Curve msg){
    curve = msg;
}

int main(int argc, char **argv){
    // Registers Node with the Master
    ros::init(argc, argv, "get_curve");
    ros::NodeHandle nh;

    // Creates Service for getting curve
    ros::ServiceServer serv = nh.advertiseService("get_curve", &comp_curves);

    // Creates subscriber for the curve
    ros::Subscriber sub = nh.subscribe("cycle/curve", 1, &get_curve);
    ros::spin();
}
