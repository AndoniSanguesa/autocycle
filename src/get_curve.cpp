#include <ros/ros.h>
#include <autocycle_extras/GetCurve.h>
#include <autocycle_extras/Curve.h>

autocycle_extras::Curve curve;
int iden = -1;

bool comp_curves(
    autocycle_extras::GetCurve::Request &req,
    autocycle_extras::GetCurve::Response &resp
){
    if(req.iden != iden){
        resp.iden = curve.iden;
        resp.length = curve.length;
        resp.xs = curve.xs;
        resp.deltas = curve.deltas;
        resp.time = curve.time;
    } else{
        resp.iden = -1;
    }
    return true;
}

void get_curve(const autocycle_extras::Curve msg){
    curve = msg;
    iden = msg.iden;
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
