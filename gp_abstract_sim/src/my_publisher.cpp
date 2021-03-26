#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>
#include <ros/ros.h>
#include <sstream>

using namespace ros;
using namespace std_msgs;

int main(int argc,char **argv){
    init(argc,argv,"gp_publisher_is_running");
    NodeHandle n;
    String  s;
    s.data = "ditto";
    int totalNumber = 4;
    Publisher velocity_publisher[totalNumber];
    int num = totalNumber;
    for(num = totalNumber ; num > 0 ; num--){
        velocity_publisher[num-1] = n.advertise<Float32>("/" + s.data + std::to_string(num) +"/left_wheel_speed", 10);
    }
    
    // velocity_publisher = n.advertise<Float32>("/" + s.data + std::to_string(num) +"/left_wheel_speed", 10);
    // Publisher velocity_publisher1 = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    // Publisher velocity_publisher2 = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    // Publisher velocity_publisher3 = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    // Publisher velocity_publisher4 = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    // velocity_publisher[1] = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    // velocity_publisher[2] = n.advertise<Float32>("/ditto1/left_wheel_speed", 10);
    
    Rate loop_rate(10);
    Float32 rakam;
    rakam.data = 0.5;
    while (ros::ok()){
        ROS_INFO("left wheel speed is %f", rakam.data);
        // spin();
        for(int num = totalNumber ; num > 0 ; num--){
      
            velocity_publisher[num-1].publish(rakam);
            ros::spinOnce();
            loop_rate.sleep();
        }
        // velocity_publisher.publish(rakam);
        
    }
    return 0;
}