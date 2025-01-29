#!/usr/bin/env python3

import math
import numpy
import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Quaternion

class GazeboUpdater(object):
    def __init__(self, model_name, pose_topic):
        self.model_name = model_name
        self.global_frame_id = "world"

        rospy.wait_for_service("gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("gazebo/set_model_state",
                                                  SetModelState, persistent=True)

        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped,
                                         self._pose_cb, queue_size=1)

    def _pose_cb(self, pose):
        new_model_state = ModelState()
        new_model_state.model_name = self.model_name
        new_model_state.pose = pose.pose
        new_model_state.reference_frame = self.global_frame_id
        self.set_model_state(new_model_state)

def main():
    rospy.init_node("gazebo_pose_updater")
    rospy.loginfo("Starting pose updater for the LIDAR emulator")
    model_name = rospy.get_param("~model_name", "robot")
    node = GazeboUpdater(model_name, "/sim/ground_truth")

    rospy.spin()

if __name__ == "__main__":
    main()