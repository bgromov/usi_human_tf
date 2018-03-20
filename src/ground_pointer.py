#!/usr/bin/env python

import copy
import math

import rospy
import tf2_ros
import tf_conversions as tfc
from geometry_msgs.msg import PoseStamped

import PyKDL as kdl

if __name__ == '__main__':
    rospy.init_node('ground_pointer')

    loop_rate = rospy.Rate(10)

    tf_buff = tf2_ros.Buffer()
    tf_ls = tf2_ros.TransformListener(tf_buff)
    # tf_br = tf2_ros.TransformBroadcaster()

    pose_pub = rospy.Publisher('pointer', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        try:
            loop_rate.sleep()
        except rospy.ROSException, e:
            if e.message == 'ROS time moved backwards':
                # rospy.logwarn('ROS time moved backwards')
                tf_buff.clear()

        try:
            eyes_tf = tf_buff.lookup_transform('human_footprint', 'eyes', rospy.Time())
            finger_tf = tf_buff.lookup_transform('human_footprint', 'finger', rospy.Time())

            eyes = kdl.Vector(eyes_tf.transform.translation.x,
                                  eyes_tf.transform.translation.y,
                                  eyes_tf.transform.translation.z)

            finger = kdl.Vector(finger_tf.transform.translation.x,
                                    finger_tf.transform.translation.y,
                                    finger_tf.transform.translation.z)

            pointing_ray = finger - eyes
            ground_normal = kdl.Vector(0.0, 0.0, 1.0)

            u = pointing_ray
            w = eyes

            D = kdl.dot(ground_normal, u)
            N = -kdl.dot(ground_normal, w)
            sI = N / D

            if sI < 0.0:
                continue

            pointer_pos = w + sI * u
            pointer_dir = copy.deepcopy(pointer_pos)
            pointer_dir.Normalize()

            pose = kdl.Frame(kdl.Rotation.RPY(0, 0,
                math.atan2(pointer_dir.y(), pointer_dir.x())), pointer_pos)

            msg = PoseStamped()
            msg.header.stamp = eyes_tf.header.stamp
            msg.header.frame_id = 'human_footprint'
            msg.pose = tfc.toMsg(pose)

            pose_pub.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, rospy.ROSException), e:
            continue