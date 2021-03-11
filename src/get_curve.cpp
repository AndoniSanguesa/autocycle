#include <ros/ros.h>
#include <autocycle/GetCurve.h>
#include <autocycle/Curve.h>

autocycle::Curve curve;
int iden = -1;

bool comp_curves(
    autocycle::GetCurve::Request &req,
    autocycle::GetCurve::Response &resp
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

void get_curve(const autocycle::Curve msg){
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
