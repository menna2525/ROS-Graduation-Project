#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <cmath>

using namespace ros;
using namespace gazebo_msgs;

struct Quat {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quat q);

int main(int argc, char **argv)
{
    init(argc, argv, "service_cl");

    NodeHandle n;

    ServiceClient client = n.serviceClient<GetModelState>("/gazebo/get_model_state");

    GetModelState mdl;

    Quat q;

    EulerAngles angles;
    // ModelState mdl;

    mdl.request.model_name = "ditto2";
    
    client.call(mdl);

    ROS_INFO("Ox: %f", mdl.response.pose.position.x);
    ROS_INFO("Oy: %f", mdl.response.pose.position.y);
    ROS_INFO("Oz: %f", mdl.response.pose.position.z);

    q.w = mdl.response.pose.orientation.w;
    q.x = mdl.response.pose.orientation.x;
    q.y = mdl.response.pose.orientation.y;
    q.z = mdl.response.pose.orientation.z;

    angles = ToEulerAngles(q);

    ROS_INFO("Wx: %f", angles.roll);
    ROS_INFO("Wy: %f", angles.pitch);
    ROS_INFO("Wz: %f", angles.yaw);
    // ROS_INFO("Wz: %f", mdl.response.pose.orientation.w);

    // ROS_INFO("Az: %f", mdl.response.twist.angular.z);

    return 0;

}

EulerAngles ToEulerAngles(Quat q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}