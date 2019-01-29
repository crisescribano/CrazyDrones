#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <ros/ros.h>

#include "mav_msgs/TorqueThrust.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <rosbag/bag.h>


rosbag::Bag bag;

//####################
//# SUBSCRIBE 2 POSE #
//####################

void receivedPose(const nav_msgs::Odometry::ConstPtr& msg){
    bag.write("crazyflie_" + msg->header.frame_id, msg->header.stamp, msg);
}

void receivedForce(const mav_msgs::TorqueThrust::ConstPtr& msg){
    bag.write("forces_crazyflie_" + msg->header.frame_id, msg->header.stamp, msg);
}

void receivedBeta(const geometry_msgs::PoseStamped::ConstPtr& msg){
	bag.write("betas_crazyflie_" + msg->header.frame_id, msg->header.stamp, msg);
}

void receivedTrajectory(const geometry_msgs::PoseStamped::ConstPtr& msg){
    bag.write("trajectory", msg->header.stamp, msg);
}

int main(int argc, char **argv)
{  
   char cwd[PATH_MAX];
   if (getcwd(cwd, sizeof(cwd)) != NULL) {
       printf("Current working dir: %s\n", cwd);
   } else {
       perror("getcwd() error");
       return 1;
   }
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "bagRecorder");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n("~");

  std::string currentTime = std::to_string(ros::Time::now().toNSec());
  
  int numQuadcopter; 
  std::string data2save;

  n.getParam("num_quads", numQuadcopter);  
  n.getParam("data2save", data2save);

  std::cout << numQuadcopter << " = NUM QUADS \n";

  bag.open("../CrazyDrones/rosbagData/data_" + currentTime + "_" + data2save + ".bag", rosbag::bagmode::Write);
 
  std::list<ros::Subscriber> subscribersList;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  for(int i = 0 ; i < numQuadcopter ; i++){
  	subscribersList.push_back(n.subscribe("/crazyflie_" + std::to_string(i) + "/out_pos_odometry", 1000, receivedPose));
	subscribersList.push_back(n.subscribe("/crazyflie_" + std::to_string(i) + "/forces_input", 1000, receivedForce));
	subscribersList.push_back(n.subscribe("/crazyflie_" + std::to_string(i) + "/betas_input", 1000, receivedBeta));
  }

  subscribersList.push_back(n.subscribe("/trajectory", 1000, receivedTrajectory));

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  bag.close();

  return 0;
}


