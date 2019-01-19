#include "cf_dynamical_model.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	// ROS set-ups:
	ros::init(argc, argv, "dynamic_model", ros::init_options::AnonymousName); //node name

	ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

	std::string topic; 
	std::string agent_number; 
	nh.getParam("topic", topic);
	nh.getParam("agent_number", agent_number);

	ROS_INFO("Initialized: %s", topic.c_str());
	ROS_INFO("Agent number: %s", agent_number.c_str());

	CF_model cf_model(&nh, topic, agent_number);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
	 
	cf_model.runDynamics();

	ros::spin();

	return 0;
}
