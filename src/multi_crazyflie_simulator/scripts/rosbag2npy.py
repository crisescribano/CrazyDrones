#!/usr/bin/env python
import rospy
import os
import yaml
import numpy as np
from rosbag.bag import Bag

statesDirectory = "../CrazyDrones/rosbagData/"
storeDirectory = "../CrazyDrones/plots/"

def main():

    rospy.init_node('converter', anonymous=True)

    directoriesList = os.listdir(statesDirectory)

    for file in directoriesList:
        if file.endswith(".bag"):
            rospy.loginfo("Reading file: " + statesDirectory + "/" + file + ". . .")
            out_dictionary = {}
            bag = Bag(statesDirectory + "/" + file)
            topics = bag.get_type_and_topic_info()[1].keys()
            minTime = 10e20
            for topic, msg, t in bag.read_messages():
                if topic.split("_")[0] == "crazyflie":
                    if topic not in out_dictionary:
                        out_dictionary[topic] = {}
                        out_dictionary[topic]["pos"] = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
                        out_dictionary[topic]["vel"] = np.array([[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]])
                        out_dictionary[topic]["att"] = np.array([[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]])
                    else:
                        out_dictionary[topic]["pos"] = np.concatenate((out_dictionary[topic]["pos"], np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])), axis = 0)
                        out_dictionary[topic]["vel"] = np.concatenate((out_dictionary[topic]["vel"], np.array([[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]])), axis = 0)
                        out_dictionary[topic]["att"] = np.concatenate((out_dictionary[topic]["att"], np.array([[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]])), axis = 0)

                if topic == "trajectory":
                    if topic not in out_dictionary:
                        out_dictionary[topic] = {}
                        out_dictionary[topic]["pos"] = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]])
                    else:
                        out_dictionary[topic]["pos"] = np.concatenate((out_dictionary[topic]["pos"], np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]])), axis = 0)

                if topic.split("_")[0] == 'betas':
                    if topic not in out_dictionary:
                        out_dictionary[topic] = {}
                        out_dictionary[topic]["beta_con"] = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]])
                        out_dictionary[topic]["beta_col"] = np.array([[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]])
                    else:
                        out_dictionary[topic]["beta_con"] = np.concatenate((out_dictionary[topic]["beta_con"], np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]])), axis = 0)
                        out_dictionary[topic]["beta_col"] = np.concatenate((out_dictionary[topic]["beta_col"], np.array([[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]])), axis = 0)

                if topic.split("_")[0] == "forces":
                    if topic not in out_dictionary:
                        out_dictionary[topic] = {}
                        out_dictionary[topic]["force"] = np.array([[msg.thrust.x, msg.thrust.y, msg.thrust.z]])
                    else:
                        out_dictionary[topic]["force"] = np.concatenate((out_dictionary[topic]["force"], np.array([[msg.thrust.x, msg.thrust.y, msg.thrust.z]])), axis = 0)

                if "time" not in out_dictionary[topic]:
                    out_dictionary[topic]["time"] = []

                out_dictionary[topic]["time"].append(t.secs + t.nsecs*1e-9)

                if min(out_dictionary[topic]["time"]) < minTime:
                    minTime = min(out_dictionary[topic]["time"])

            for topic in topics:
                out_dictionary[topic]["time"] = [(t - minTime) for t in out_dictionary[topic]["time"]]

            rospy.loginfo("Storing data in file: " + storeDirectory + "/" + file.split(".bag")[0] + ".npy" + ". . .")
            np.save(storeDirectory + "/" + file.split(".bag")[0] + ".npy", out_dictionary)




if __name__ == '__main__':
    main()
