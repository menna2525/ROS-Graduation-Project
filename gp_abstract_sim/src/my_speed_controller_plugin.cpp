#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
class ModelJointControler : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelJointControler::OnUpdate, this));

    this->old_secs = ros::Time::now().toSec();

    //this->fixedJoint =
    /////////////////////////////////////////
    this->startTime = ros::Time::now().toSec();
    // this->model->GetLink("front_face")->GetInertial()->SetMass(0.5);
    // this->model->GetLink("front_face")->UpdateMass();

    /////////////////////////////////////////

    if (_sdf->HasElement("wheel_kp"))
      this->wheel_kp = _sdf->Get<double>("wheel_kp");
    if (_sdf->HasElement("wheel_ki"))
      this->wheel_ki = _sdf->Get<double>("wheel_ki");
    if (_sdf->HasElement("wheel_kd"))
      this->wheel_kd = _sdf->Get<double>("wheel_kd");
    if (_sdf->HasElement("namespace_model"))
      this->namespace_model = _sdf->Get<std::string>("namespace_model");
    if (_sdf->HasElement("activate_pid_control"))
      this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");

    // Create a topic name
    std::string left_wheel_speed = "/" + this->model->GetName() + "/left_wheel_speed";
    std::string right_wheel_speed = "/" + this->model->GetName() + "/right_wheel_speed";
    std::string front_face = "/" + this->model->GetName() + "/front_face";
    std::string face_pan= "/" + this->model->GetName() + "/face_pan";

    magnet_topic = "/" + this->model->GetName() + "/attach";

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "set_wheelSpeed_rosnode",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));

    if (this->activate_pid_control)
    {
      // Activated PID Speed Control
      const auto &jointController = this->model->GetJointController();
      jointController->Reset();

      jointController->AddJoint(model->GetJoint("right_wheel_joint"));
      jointController->AddJoint(model->GetJoint("left_wheel_joint"));
      jointController->AddJoint(model->GetJoint("pan_joint"));


      this->right_wheel_name = model->GetJoint("right_wheel_joint")->GetScopedName();
      this->left_wheel_name = model->GetJoint("left_wheel_joint")->GetScopedName();
      this->pan_face_name = model->GetJoint("pan_joint")->GetScopedName();


      jointController->SetVelocityPID(this->right_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
      jointController->SetVelocityPID(this->left_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
      jointController->SetVelocityPID(this->pan_face_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));

      jointController->SetVelocityTarget(this->right_wheel_name, 0.0);
      jointController->SetVelocityTarget(this->left_wheel_name, 0.0);
      jointController->SetVelocityTarget(this->pan_face_name, 0.0);

    }

    // Freq
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            left_wheel_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_left_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));
////////// subscribe pan
ros::SubscribeOptions so5 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            face_pan,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_pan_face, this, _1),
            ros::VoidPtr(), &this->rosQueue5);
    this->rosSub5 = this->rosNode->subscribe(so5);

    // Spin up the queue helper thread.
    this->rosQueueThread5 =
        std::thread(std::bind(&ModelJointControler::QueueThread5, this));

////////////////////end
    // Magnitude
    ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            right_wheel_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_right_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue2);
    this->rosSub2 = this->rosNode->subscribe(so2);

    // Spin up the queue helper thread.
    this->rosQueueThread2 =
        std::thread(std::bind(&ModelJointControler::QueueThread2, this));

    //////////////////////////////////////////////////////////////////////////
    //front_face
    ros::SubscribeOptions so3 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            front_face,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_up_face, this, _1),
            ros::VoidPtr(), &this->rosQueue3);
    this->rosSub3 = this->rosNode->subscribe(so3);

    // Spin up the queue helper thread.
    this->rosQueueThread3 =
        std::thread(std::bind(&ModelJointControler::QueueThread3, this));

    //get magnetized
    ros::SubscribeOptions so4 =
        ros::SubscribeOptions::create<std_msgs::String>(
            magnet_topic,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_attach, this, _1),
            ros::VoidPtr(), &this->rosQueue4);
    this->rosSub4 = this->rosNode->subscribe(so4);

    // Spin up the queue helper thread.
    this->rosQueueThread4 =
        std::thread(std::bind(&ModelJointControler::QueueThread4, this));
    ////////////////////////////////////////////////////////////////////////////

    ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    double new_secs = ros::Time::now().toSec();
    double delta = new_secs - this->old_secs;

    double max_delta = 0.0;

    if (this->freq_update != 0.0)
    {
      max_delta = 1.0 / this->freq_update;
    }

    if (delta > max_delta && delta != 0.0)
    {
      this->old_secs = new_secs;

      if (this->activate_pid_control)
      {
        ROS_DEBUG("Update Wheel Speed PID...");
        const auto &jointController = this->model->GetJointController();
        jointController->SetVelocityTarget(this->right_wheel_name, this->right_wheel_speed_magn);
        jointController->SetVelocityTarget(this->left_wheel_name, this->left_wheel_speed_magn);
        jointController->SetVelocityTarget(this->pan_face_name, this->pan_face_speed_magn);

      }
      else
      {
        //   if(counterflag == 0){
        //     counterflag++;
        //     sleep(2);
        // }

        // Apply a small linear velocity to the model.
        ROS_DEBUG("Update Wheel Speed BASIC...");
        this->model->GetJoint("right_wheel_joint")->SetParam("fmax", 0, 10.0);
        this->model->GetJoint("right_wheel_joint")->SetParam("vel", 0, this->right_wheel_speed_magn);
        this->model->GetJoint("left_wheel_joint")->SetParam("fmax", 0, 10.0);
        this->model->GetJoint("left_wheel_joint")->SetParam("vel", 0, this->left_wheel_speed_magn);
        // this->model->GetJoint("right_wheel_joint")->SetVelocity(0, this->right_wheel_speed_magn);
        // this->model->GetJoint("left_wheel_joint")->SetVelocity(0, this->left_wheel_speed_magn);
        this->model->GetJoint("tilt_joint")->SetLowerLimit(0, 0);
        this->model->GetJoint("tilt_joint")->SetUpperLimit(0, 1.5707);
        this->model->GetJoint("tilt_joint")->SetParam("fmax", 0, 50.0);
        // this->model->GetJoint("face_mover_joint")->SetVelocity(0, -0.1);
        /////////// pan joint 

       //this->model->GetJoint("pan_joint")->SetLowerLimit(0, -0.785398);
       //this->model->GetJoint("pan_joint")->SetUpperLimit(0, 0.785398);
        this->model->GetJoint("pan_joint")->SetParam("fmax", 0, 50.0);
        if ((new_secs - this->startTime) > 10.0)
        {
          // ROS_INFO("sec is :%f",(new_secs-this->startTime));
          if ((this->model->GetName() == "dittaya") && (this->counterflag == 0))
          {
            this->counterflag = 1;
            ROS_INFO("This is dittaya");
            this->worldfr = this->model->GetWorld();
            physics::ModelPtr mdlptr = this->worldfr->ModelByName("ditto2");
            if (mdlptr->GetName() == "ditto2")
              ROS_INFO("we have ditto2");
            this->someLink = mdlptr->GetLink("body");
            if (someLink->GetName() == "body")
              ROS_INFO("we have body");
            this->fixedJoint = this->model->CreateJoint("fixedjoint", "revolute", this->model->GetLink("front_face"), this->someLink);
            this->fixedJoint->Init();
            this->fixedJoint->SetLowerLimit(0, 0);
            this->fixedJoint->SetUpperLimit(0, 0);
            this->model->GetJoint("tilt_joint")->SetParam("vel", 0, 0.1);
            this->model->GetJoint("pan_joint")->SetParam("vel", 0, 0.1);

          }
          // this->model->GetLink("front_face")->GetInertial()->SetMass(150.0);
          // this->model->GetLink("front_face")->UpdateMass();
          this->model->GetJoint("tilt_joint")->SetParam("vel", 0, front_face_speed_magn);
          this->model->GetJoint("pan_joint")->SetParam("vel", 0, pan_face_speed_magn);


        }
      }
    }
  }

public:
  void SetLeftWheelSpeed(const double &_freq)
  {
    this->left_wheel_speed_magn = _freq;
    ROS_WARN("left_wheel_speed_magn >> %f", this->left_wheel_speed_magn);
  }

public:
  void SetRightWheelSpeed(const double &_magn)
  {
    this->right_wheel_speed_magn = _magn;
    ROS_WARN("right_wheel_speed_magn >> %f", this->right_wheel_speed_magn);
  }

public:
  void OnRosMsg_left_wheel_speed(const std_msgs::Float32ConstPtr &_msg)
  {
    this->SetLeftWheelSpeed(_msg->data);
  }
///////////////////////////////////////////////////////////////////////////////////////////
public:
  void OnRosMsg_up_face(const std_msgs::Float32ConstPtr &_msg)
  {
    this->SetFrontFace_up(_msg->data);
  }
  /////////////pan on Ros
  public:
  void SetPanFaceSpeed(const double &_magn)
  {
    this->pan_face_speed_magn = _magn;
    ROS_WARN("rotataing_face_speed_magn >> %f", this->pan_face_speed_magn);
  }

  public:
  void OnRosMsg_pan_face(const std_msgs::Float32ConstPtr &_msg)
  {
    this->SetPanFaceSpeed(_msg->data);
  }


public:
  void SetFrontFace_up(const double &_freq)
  {
    this->front_face_speed_magn = _freq;
    ROS_WARN("front_face_speed_magn >> %f", this->front_face_speed_magn);
  }

  public:
  void OnRosMsg_attach(const std_msgs::StringConstPtr &_msg)
  {
    std_msgs::String sendastring;
    sendastring.data = _msg->data;
    ROS_INFO("%s",_msg->data.c_str());
    ROS_INFO("%s",sendastring.data.c_str());
    this->magnetism(sendastring);
  }

  public:
  void magnetism(std_msgs::String faceToAttach)
  {
    // ROS_INFO("%s",faceToAttach.data.c_str());
    int pos = faceToAttach.data.find_first_of('#');
    std::string firstname = faceToAttach.data.substr(0, pos),
      lastname = faceToAttach.data.substr(pos+1);
    // ROS_INFO("%s%s",firstname.c_str(),lastname.c_str());
    int pos2 = lastname.find_first_of('/');
    std::string firstname2 = lastname.substr(0, pos2),
      lastname2 = lastname.substr(pos2+1);
      ROS_INFO("failed here 0");
      this->worldfr = this->model->GetWorld();
    physics::ModelPtr mdlptrToModule = this->worldfr->ModelByName(firstname2);
    ROS_INFO("failed here 00");
    // ROS_INFO("%s%s%s",firstname.c_str(),firstname2.c_str(),lastname2.c_str());
    ////////// this-model->RelativePose;
    if (mdlptrToModule->GetName() == firstname2)
    { ROS_INFO("failed here 1");
      this->someLink = mdlptrToModule->GetLink(lastname2);
      ROS_INFO("failed here 2");
      if (someLink->GetName() == lastname2)
      {
        if((firstname == "body")&&(isattached_body)){
          return;
        }
        if((firstname == "front_face")&&(isattached_front_face)){
          return;
        }
        if((firstname == "left_wheel")&&(isattached_left_wheel)){
          return;
        }
        if((firstname == "right_wheel")&&(isattached_right_wheel)){
          return;
        }

        if(firstname == "body"){
          isattached_body = true;
        }
        if(firstname == "front_face"){
          isattached_front_face = true;
        }
        if(firstname == "left_wheel"){
          isattached_left_wheel = true;
        }
        if(firstname == "right_wheel"){
          isattached_right_wheel = true;
        }

        ROS_INFO("failed here 3");
        this->fixedJoint = this->model->CreateJoint("fixedjoint" + firstname, "revolute", this->model->GetLink(firstname), this->someLink);
        ROS_INFO("failed here 4");
        this->fixedJoint->Init();
        this->fixedJoint->SetLowerLimit(0, 0);
        this->fixedJoint->SetUpperLimit(0, 0);
      }
    }
  }
///////////////////////////////////////////////////////////////////////////////////////////
  /// \brief ROS helper function that processes messages
private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

public:
  void OnRosMsg_right_wheel_speed(const std_msgs::Float32ConstPtr &_msg)
  {
    this->SetRightWheelSpeed(_msg->data);
  }

  

  /// \brief ROS helper function that processes messages
private:
  void QueueThread2()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue2.callAvailable(ros::WallDuration(timeout));
    }
  }

///////////////////////////////////////////////////////////////////
private:
  void QueueThread3()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue3.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  void QueueThread4()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue4.callAvailable(ros::WallDuration(timeout));
    }
  }
  //pan queue


private:
  void QueueThread5()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue5.callAvailable(ros::WallDuration(timeout));
    }
  }
/////////////////////////////////////////////////////////////////



  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;

  // Time Memory
  double old_secs;

  // Frequency of earthquake
  double freq_update = 10.0;

  double left_wheel_speed_magn = 0.0;
  // Magnitude of the Oscilations
  double right_wheel_speed_magn = 0.0;
///////////////////////////////////////////////////////////////////////////////
  double front_face_speed_magn = 0.0;
///////////////////////////////////////////////////////////////////////////////
  double pan_face_speed_magn = 0.0;

///////////////////
  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub2;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue2;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread2;

/////////////////////////////////////////////////////////////////////////////
  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub3;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue3;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread3;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub4;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue4;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread4;
  ///////////////////////////////////////////////////////////////////////////
  // for pan movement 
  private:
  ros::Subscriber rosSub5;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue5;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread5;

  std::string right_wheel_name;
  std::string left_wheel_name;
  std::string magnet_topic;
  std::string pan_face_name;


  std::string namespace_model = "";
  bool activate_pid_control;

  double wheel_kp = 0.1;
  double wheel_ki = 0.0;
  double wheel_kd = 0.0;

  /////////////////////////////////////////
private:
  physics::JointPtr fixedJoint;

private:
  physics::LinkPtr someLink;

private:
  physics::WorldPtr worldfr;

private:
  int counterflag = 0;

private:
  double startTime = 0;

private:
  bool isattached_front_face = false;
  bool isattached_body = false;
  bool isattached_right_wheel = false;
  bool isattached_left_wheel = false;
  bool can_attach = false;
  bool join_flag = false;

  /////////////////////////////////////////
}; // namespace gazebo

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
} // namespace gazebo
