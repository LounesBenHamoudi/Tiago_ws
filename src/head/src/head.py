#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import sys

class AutoRelocaliser:
    def __init__(self):
        rospy.init_node('auto_relocaliser_at_goal')

        # Action client pour move_base
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connexion à move_base...")
        self.client.wait_for_server()
        rospy.loginfo(" Connecté à move_base !")

        # Publisher vers /initialpose
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    def send_goal(self, x, y, theta):
        rospy.loginfo("Goal demandé : x=%.2f, y=%.2f, theta=%.2f (rad)", x, y, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(" Goal atteint avec succès !")
            self.publish_initial_pose(x, y, theta)
        else:
            rospy.logwarn("Le goal n'a pas été atteint (statut = %d)", state)

    def publish_initial_pose(self, x, y, theta):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        pose.pose.covariance = [0.01] * 36
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.05

        self.initial_pose_pub.publish(pose)

        rospy.loginfo("Position publiée sur /initialpose :")
        rospy.loginfo("     x = %.2f", pose.pose.pose.position.x)
        rospy.loginfo("     y = %.2f", pose.pose.pose.position.y)
        rospy.loginfo("     θ = %.2f rad", theta)

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage : rosrun ton_package auto_relocalise_at_goal.py x y theta")
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])

        node = AutoRelocaliser()
        node.send_goal(x, y, theta)

    except rospy.ROSInterruptException:
        rospy.loginfo(" Noeud interrompu.")
