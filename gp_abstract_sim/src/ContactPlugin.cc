#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
	// Get the parent sensor.
	this->parentSensor =
		std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

	// Make sure the parent sensor is valid.
	if (!this->parentSensor)
	{
		// gzerr << "ContactPlugin requires a ContactSensor.\n";
		ROS_WARN("ContactPlugin requires a ContactSensor.");
		return;
	}

	// Connect to the sensor update event.
	this->updateConnection = this->parentSensor->ConnectUpdated(
		std::bind(&ContactPlugin::OnUpdate, this));

	// Make sure the parent sensor is active.
	this->parentSensor->SetActive(true);

	if (_sdf->HasElement("magnetic_face"))
	{
		this->magnetic_face = _sdf->Get<std::string>("magnetic_face");
		ROS_WARN("Loaded ContactPlugin with parent...%s", magnetic_face.c_str());
	}

	parent = _sensor->ParentName().substr(0, 6);
	ROS_WARN("sensor parent...%s", parent.c_str());

	pub = n.advertise<std_msgs::String>("sensor_data", 10);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
	// Get all the contacts.
	ros::Rate loop_rate(10);
	msgs::Contacts contacts;
	contacts = this->parentSensor->Contacts();
	for (unsigned int i = 0; i < contacts.contact_size(); ++i)
	{
		// std::cout << "Collision between[" << contacts.contact(i).collision1()
		//           << "] and [" << contacts.contact(i).collision2() << "]\n";
		ROS_WARN("Collision between[%s] and [%s]", contacts.contact(i).collision1().c_str(), contacts.contact(i).collision2().c_str());
		ROS_INFO("Hello you");
		if (contacts.contact(i).collision1().find(parent) != std::string::npos)
		{
			//////////////////
			ROS_INFO("Hello");
			if (contacts.contact(i).collision1().find("body_magnet") != std::string::npos)
			{
				///////////////////parent north
				if (contacts.contact(i).collision1().find("north") != std::string::npos)
				{
					if (contacts.contact(i).collision2().find("body_magnet") != std::string::npos)

					{
						if (contacts.contact(i).collision2().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/body_magnet_north");
						}
						else if (contacts.contact(i).collision2().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/body_magnet_south");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
							pub.publish(col_faces);
						}
					}

					else if (contacts.contact(i).collision2().find("front_face_magnet") != std::string::npos)
					{
						if (contacts.contact(i).collision2().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/front_face_magnet_north");
							//testing
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision2().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/front_face_magnet_south");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
					}
					////////////////// parent south
				}
				else if (contacts.contact(i).collision1().find("south") != std::string::npos)
				{
					if (contacts.contact(i).collision2().find("body_magnet") != std::string::npos)

					{
						if (contacts.contact(i).collision2().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/body_magnet_north");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision2().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/body_magnet_south");
						}
					}

					else if (contacts.contact(i).collision2().find("front_face_magnet") != std::string::npos)
					{
						if (contacts.contact(i).collision2().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/front_face_magnet_north");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision2().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/front_face_magnet_south");
							//testing south
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
					}
				}
				//////////////parent is front face
				else if (contacts.contact(i).collision1().find("front_face_magnet") != std::string::npos)
				{
					///////////////////parent north
					if (contacts.contact(i).collision1().find("north") != std::string::npos)
					{
						if (contacts.contact(i).collision2().find("body_magnet") != std::string::npos)

						{
							if (contacts.contact(i).collision2().find("north") != std::string::npos)
							{
								/////testing
								ROS_INFO("front_face_magnet_north/body_magnet_north");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision2().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/body_magnet_south");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
						}

						else if (contacts.contact(i).collision2().find("front_face_magnet") != std::string::npos)
						{
							if (contacts.contact(i).collision2().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/front_face_magnet_north");
							}
							else if (contacts.contact(i).collision2().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/front_face_magnet_south");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
								pub.publish(col_faces);
							}
						}
						////////////////// parent south
					}
					else if (contacts.contact(i).collision1().find("south") != std::string::npos)
					{
						if (contacts.contact(i).collision2().find("body_magnet") != std::string::npos)

						{
							if (contacts.contact(i).collision2().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/body_magnet_north");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision2().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/body_magnet_south");
								//testing south
								col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
						}

						else if (contacts.contact(i).collision2().find("front_face_magnet") != std::string::npos)
						{
							if (contacts.contact(i).collision2().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/front_face_magnet_north");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision2().substr(0, 6) + "/front_face";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision2().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/front_face_magnet_south");
							}
						}
					}
				}
			}
		}

		//////////////////////////////////////////////////parent is body parent is collison 2
		else if (contacts.contact(i).collision2().find(parent) != std::string::npos)
		{
			//////////////////
			if (contacts.contact(i).collision2().find("body_magnet") != std::string::npos)
			{
				///////////////////parent north
				if (contacts.contact(i).collision2().find("north") != std::string::npos)
				{
					if (contacts.contact(i).collision1().find("body_magnet") != std::string::npos)

					{
						if (contacts.contact(i).collision1().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/body_magnet_north");
						}
						else if (contacts.contact(i).collision1().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/body_magnet_south");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
							pub.publish(col_faces);
						}
					}

					else if (contacts.contact(i).collision1().find("front_face_magnet") != std::string::npos)
					{
						if (contacts.contact(i).collision1().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_north/front_face_magnet_north");
							//testing
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision1().find("south") != std::string::npos)
						{

							ROS_INFO("body_magnet_north/front_face_magnet_south");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
					}
					////////////////// parent south
				}
				else if (contacts.contact(i).collision2().find("south") != std::string::npos)
				{
					if (contacts.contact(i).collision1().find("body_magnet") != std::string::npos)

					{
						if (contacts.contact(i).collision1().find("north") != std::string::npos)
						{

							ROS_INFO("body_magnet_south/body_magnet_north");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision1().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/body_magnet_south");
						}
					}

					else if (contacts.contact(i).collision1().find("front_face_magnet") != std::string::npos)
					{
						if (contacts.contact(i).collision1().find("north") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/fornt_face_magnet_north");
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
						else if (contacts.contact(i).collision1().find("south") != std::string::npos)
						{
							ROS_INFO("body_magnet_south/fornt_face_magnet_south");
							//testing south
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
							pub.publish(col_faces);
						}
					}
				}
				//////////////parent is front face collision 2
				else if (contacts.contact(i).collision2().find("front_face_magnet") != std::string::npos)
				{
					///////////////////parent north
					if (contacts.contact(i).collision2().find("north") != std::string::npos)
					{
						if (contacts.contact(i).collision1().find("body_magnet") != std::string::npos)

						{
							if (contacts.contact(i).collision1().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/body_magnet_north");
								////testing
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision1().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/body_magnet_south");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
						}

						else if (contacts.contact(i).collision1().find("front_face_magnet") != std::string::npos)
						{
							if (contacts.contact(i).collision1().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/front_face_magnet_north");
							}
							else if (contacts.contact(i).collision1().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_north/front_face_magnet_south");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
								pub.publish(col_faces);
							}
						}
						////////////////// parent south
					}
					else if (contacts.contact(i).collision2().find("south") != std::string::npos)
					{
						if (contacts.contact(i).collision1().find("body_magnet") != std::string::npos)

						{
							if (contacts.contact(i).collision1().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/body_magnet_north");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision1().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/body_magnet_south");
								//testing south
							col_faces.data = parent + "/" + "body" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/body";
							pub.publish(col_faces);
							}
						}

						else if (contacts.contact(i).collision1().find("front_face_magnet") != std::string::npos)
						{
							if (contacts.contact(i).collision1().find("north") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/front_face_magnet_north");
								col_faces.data = parent + "/" + "front_face" + "#" + contacts.contact(i).collision1().substr(0, 6) + "/front_face";
								pub.publish(col_faces);
							}
							else if (contacts.contact(i).collision1().find("south") != std::string::npos)
							{
								ROS_INFO("front_face_magnet_south/front_face_magnet_south");
							}
						}
					}
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
