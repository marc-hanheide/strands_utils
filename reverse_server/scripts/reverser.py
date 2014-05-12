#! /usr/bin/env python

import roslib
roslib.load_manifest('reverse_server')
import rospy

import actionlib
import actionlib_msgs.msg

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import reverse_server.msg

class Reverser(object):

    def __init__(self, name):
        #self._memory_game_pkg = rospy.get_param(
        #    '~memory_game_pkg', 'memory_launch')
        #self._memory_game_file = rospy.get_param(
        #    '~memory_game_file', 'memory.launch')


        # Waits until the action server has started up and started
        # listening for goals.
        self._action_name = name
        self._goal_dist = 0.5
        self._as = actionlib.SimpleActionServer(
            self._action_name, reverse_server.msg.reverseAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._last_odom = None

        self._as.start()

    def odom_cb(self, msg):
        rospy.logdebug("odometry received")
        self._last_odom = msg.pose.pose.position

    def distance(self, p1, p2):
        return ((p1.x - p2.x) ** 2
                + (p1.y - p2.y) ** 2
                + (p1.z - p2.z) ** 2) ** 0.5

    def execute_cb(self, goal):
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self._twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        while not self._as.is_preempt_requested() and self._last_odom is None:
            rospy.sleep(rospy.Duration(secs=1, nsecs=100000))
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return
        start_pos = self._last_odom
        rospy.loginfo("starting position initialised")


        while self.distance(start_pos, self._last_odom) < goal.distance and not self._as.is_preempt_requested() and not rospy.is_shutdown():
            t = Twist()
            t.linear.x = -0.1 # go slowly!
            self._twist_pub.publish(t)
            rospy.logdebug("current distance is " + str(self.distance(start_pos, self._last_odom)))
            rospy.sleep(rospy.Duration(secs=0, nsecs=20000))
        rospy.loginfo("stop")

        t = Twist()
        self._twist_pub.publish(t)

        if not self.distance(start_pos, self._last_odom) < self._goal_dist:
            rospy.loginfo("position reached")
            self._as.set_succeeded()
            return
        if self._as.is_preempt_requested():
            rospy.loginfo("reversing preempted")
            self._as.set_preempted()
            return
        rospy.loginfo("reversing aborted")
        self._as.set_aborted()
        return

if __name__ == '__main__':
    rospy.init_node('reverser')
    Reverser(rospy.get_name())
    rospy.spin()
