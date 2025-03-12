#!/usr/bin/env python3
import rospy

import math
import tf2_ros
import tf
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray

import numpy

def dist(pose):
    return math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)

def main():
    rospy.init_node('apriltag_pose_publisher')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    camera_frame = rospy.get_param("~camera_frame_id", "arducam")
    robot_frame = rospy.get_param("~robot_frame_id", "base_footprint")
    dist_err_per_m = float(rospy.get_param("~dist_err_per_m", "0.1"))
    angle_err_per_m = float(rospy.get_param("~angle_err_per_m", "0.3"))
    x_y_err_ratio = float(rospy.get_param("~x_y_err_ratio", "4.0"))
    multitag_factor = float(rospy.get_param("multitag_factor", "0.5"))

    pub = rospy.Publisher("apriltag/pose", PoseWithCovarianceStamped, queue_size=1)

    def detection_callback(detections):
        if len(detections.detections) == 0:
            return

        t = detections.header.stamp
        saw_field = False

        detections = detections.detections
        closest_tag_pose = None
        total_dist = 0
        closest_dist = 99999999
        num_tags = 0
        for d in detections:
            if len(d.id) == 1:
                num_tags += 1
                tag_dist = dist(d.pose.pose.pose)
                total_dist += tag_dist
                if tag_dist < closest_dist:
                    closest_tag_pose = d.pose.pose.pose
                    closest_dist = tag_dist
            elif len(d.id) > 6:
                saw_field = True

        if num_tags < 1:
            return

        avg_dist = total_dist / num_tags

        dist_score = 0.67*closest_dist + 0.33*avg_dist
        if num_tags > 2:
            dist_score *= multitag_factor
        # every 1m away 3cm of error std_dev we'll say
        a = dist_err_per_m*dist_score
        aa = a*a

        # every 1m away 0.1 radians of error std_dev we'll say
        b = angle_err_per_m*dist_score
        bb = b*b

        detection_covariance = numpy.array([
            [x_y_err_ratio*aa,0, 0, 0, 0, 0], # Harder to estimate distance to tags
            [0, aa,0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0 ,0, 0, 0, bb],
        ])

        robot_transform = tfBuffer.lookup_transform("full_field", robot_frame, t, rospy.Duration(1.0))
        robot_transform = robot_transform.transform # dont need stamp

        quat = robot_transform.rotation
        yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        c = math.cos(yaw)
        s = math.sin(yaw)

        rotation = numpy.array([
            [c, -s, 0, 0, 0, 0],
            [s, c, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        covariance = rotation @ detection_covariance @ rotation.T

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = t
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = robot_transform.translation.x
        msg.pose.pose.position.y = robot_transform.translation.y
        msg.pose.pose.position.z = robot_transform.translation.z
        msg.pose.pose.orientation = robot_transform.rotation
        msg.pose.covariance = covariance.flatten().tolist()

        pub.publish(msg)

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, detection_callback, queue_size=1)

    rospy.spin()



if __name__ == '__main__':
    main()