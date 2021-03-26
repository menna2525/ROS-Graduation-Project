#include <sstream>
#include <termios.h>
#include <cmath>
// #include <curses.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
// #include "qt5/QtCore/qmap.h"
// #include <QKeyEvent>
// #include <QLabel>
// #include <QPushButton>
// #include <QtGui/QGuiApplication>

using namespace ros;
using namespace std_msgs;

int caseInput = 's';

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc,char **argv)
{

    init(argc,argv,"readwrite_keys");
    NodeHandle n;
    // NodeHandle l;
    ros::Rate loop_rate(40);
    String  s;
    s.data = "ditto";
    Publisher velocity_publisher1,velocity_publisher2,velocity_publisher3,velocity_publisher4;
    std::string inputString;
    velocity_publisher1 = n.advertise<Float32>("/" + s.data + std::to_string(2) +"/left_wheel_speed", 10);
    velocity_publisher2 = n.advertise<Float32>("/" + s.data + std::to_string(2) +"/right_wheel_speed", 10);
    velocity_publisher3 = n.advertise<Float32>("/" + s.data + std::to_string(2) +"/front_face", 10);
    velocity_publisher4 = n.advertise<Float32>("/" + s.data + std::to_string(2) +"/face_pan", 10);

//    velocity_publisher1 = n.advertise<Float32>("/" + s.data +"/left_wheel_speed", 10);
//    velocity_publisher2 = n.advertise<Float32>("/" + s.data +"/right_wheel_speed", 10);
//    velocity_publisher3 = n.advertise<Float32>("/" + s.data +"/front_face", 10);
    Float32 leftWheelMag,rightWheelMag,front_faceMag,face_panMag;
    while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    
    std::cin.clear(); 
    //// std::cin.ignore(INT_MAX,'\n'); 
    // std::getline(std::cin, inputString);
    caseInput = getch();
    inputString = caseInput;
    switch (caseInput)
    {
    case 'w':
      leftWheelMag.data = -10;
      rightWheelMag.data = -0.5;
      break;

    case 'a':
      leftWheelMag.data = -0.5;
      rightWheelMag.data = 0.5;
      break;

    case 'd':
      leftWheelMag.data = 0.5;
      rightWheelMag.data = -0.5;
      break;

    case 'x':
      leftWheelMag.data = 0.5;
      rightWheelMag.data = 0.5;
      break;

    case 'q':
      leftWheelMag.data = -0.75;
      rightWheelMag.data = -0.25;
      break;

    case 'e':
      leftWheelMag.data = -0.25;
      rightWheelMag.data = -0.75;
      break;

    case 'z':
      leftWheelMag.data = 0.75;
      rightWheelMag.data = 0.25;
      break;

    case 'c':
      leftWheelMag.data = 0.25;
      rightWheelMag.data = 0.75;
      break;

    case 'u':
      front_faceMag.data = 0.1;
      break;

    case 'j':
      front_faceMag.data = -0.1;
      break;

    case 'm':
      front_faceMag.data = 0.0;
      break;
    case 'o':
      face_panMag.data = 0.1;
      break;

    case 'l':
      face_panMag.data = -0.1;
      break;

     case 'i':
      face_panMag.data = 0;
      break;
      

    default:
      leftWheelMag.data = 0.0;
      rightWheelMag.data = 0.0;
      break;
    }
    
    ss << inputString;
    msg.data = ss.str();
    
    ROS_INFO("%s", msg.data.c_str());
    velocity_publisher1.publish(leftWheelMag);
    velocity_publisher2.publish(rightWheelMag);
    velocity_publisher3.publish(front_faceMag);
    velocity_publisher4.publish(face_panMag);

    ros::spinOnce();

    loop_rate.sleep();
    
  }


  return 0;
  

}
