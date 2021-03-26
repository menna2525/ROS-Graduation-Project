#include <sstream>
#include <termios.h>
#include <cmath>
// #include <curses.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>

using namespace ros;
using namespace std_msgs;

std::string firstPart,secondPart,thirdPart,fourthPart;
bool started = false;

void readTheString(const std_msgs::String::ConstPtr& msg){
    int pos = msg->data.find_first_of('#');
    std::string firstname = msg->data.substr(0,pos),
      lastname = msg->data.substr(pos+1);
      int pos2 = firstname.find_first_of('/');
      int pos3 = lastname.find_first_of('/');
    firstPart = firstname.substr(0,pos2);
    secondPart = firstname.substr(pos2+1);
    // ROS_INFO("I heard: [%s]", firstPart.c_str());
    
    // secondPart = firstname.
    thirdPart = lastname.substr(0,pos3);
    fourthPart = lastname.substr(pos3+1);
    // ROS_INFO("I heard: [%s]", lastname.c_str());
    // ROS_INFO("I heard: [%s]", firstPart.substr(0,6).c_str());
    // ROS_INFO("I heard: [%s]", secondPart.c_str());
    // ROS_INFO("I heard: [%s]", thirdPart.c_str());
    started = true;
    // std::string output_str = firstname + " " + lastname;
    // ROS_INFO("I heard: [%s]", output_str.c_str());
    ROS_INFO("I heard: [%s]", firstPart.c_str());
    ROS_INFO("I heard: [%s]", secondPart.c_str());
    ROS_INFO("I heard: [%s]", thirdPart.c_str());
    ROS_INFO("I heard: [%s]", fourthPart.c_str());
    
}

int main(int argc,char **argv)
{

    init(argc,argv,"readAndCut");
    
    // NodeHandle l;
    NodeHandle n;
    ros::Rate loop_rate(40);
    String  s;
    s.data = "hamada";
    Subscriber sub;

    Publisher pub;
    
    
    
    sub = n.subscribe("sensor_data",1000,readTheString);
    while (ros::ok())
  {
    if(started){
      pub = n.advertise<String>("/" + firstPart  +"/attach", 10);
      s.data = secondPart + "#" + thirdPart + "/" + fourthPart;
      pub.publish(s);
    }
    spinOnce();
    loop_rate.sleep();
  }

  return 0;
    
}