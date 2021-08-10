#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import serial

class TwistToMotors():

    def __init__(self):

        rospy.init_node("twist_to_motor_node")
        node_name = rospy.get_name()

        rospy.loginfo("%s started" % node_name)
        print("%s started", node_name)

        # Reading the params
        self.twist_topic = rospy.get_param("~twist_topic", "")
        self.rate = rospy.get_param("~rate", 50)
        self.w = rospy.get_param("~base_width", 0.3) # width of the robot
        self.max_pwm = rospy.get_param("~max_pwm", 0) # maximum PWM strength to send to the robot
        self.min_pwm = rospy.get_param("~min_pwm_carpet", 0) # minimum PWM strength to send to the robot
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

        self.max_vel_x = rospy.get_param("/move_base/OpenBotTrajectoryPlanner/max_vel_x", 1.0) # maximum linear velocity
        self.min_vel_x = rospy.get_param("/move_base/OpenBotTrajectoryPlanner/min_vel_x", 0.0) # minimum linear velocity
        self.max_rotational_vel = rospy.get_param("/move_base/OpenBotTrajectoryPlanner/max_rotational_vel", 1.5) # maximum rotational velocity

        self.max_input_vel =  self.max_vel_x + self.max_rotational_vel * self.w / 2 # maximum of the twist interval that we map from
        self.min_input_vel = self.min_vel_x # minimum of the twist interval that we map from

        rospy.loginfo("MAX/MIN Vel, MAX Rot: (%f, %f), %f", self.max_vel_x, self.min_vel_x, self.max_rotational_vel)

        # Subscriber
        rospy.Subscriber(self.twist_topic, Twist, self.twistCallback)

        # Global variables for storing motor commands
        self.left = 0
        self.right = 0

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)

    def close_connection(self):
        self.ser.close()

    def spin(self):

        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        # main loop
        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def spinOnce(self):

        # dx = (l + r) / 2
        # dr = (r - l) / w

        self.right = self.dx + self.dr * self.w / 2
        self.left = self.dx - self.dr * self.w / 2

        # Save the sign of the values
        right_sgn = math.copysign(1, self.right)
        left_sgn = math.copysign(1, self.left)

        rospy.loginfo("Raw twist: (%d, %d)", self.left, self.right)

        if self.left!=0.0 or self.right!=0.0:
            
            # Translate abs value of input twist into MIN_PWM and MAX_PWM interval
            self.right = self.translate(abs(self.right), self.min_input_vel, self.max_input_vel, self.min_pwm, self.max_pwm)
            self.left = self.translate(abs(self.left), self.min_input_vel, self.max_input_vel, self.min_pwm, self.max_pwm)

            # Regain the correct signs
            self.right *= right_sgn
            self.left *= left_sgn

        self.right = int(self.right)
        self.left = int(self.left)

        rospy.loginfo("PWM: (%d, %d)", self.left, self.right)

        # Send to Arduino
        #with serial.Serial('/dev/ttyUSB0', 115200, timeout=10) as ser:
        self.ser.write(bytes('c'+str(self.left)+','+str(self.right)+'\n', 'utf-8'))

        self.ticks_since_target += 1

    def twistCallback(self, msg):

        # rospy.loginfo("-D- twistCallback: %s" % str(msg))

        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)


if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        twistToMotors.close_connection()
        pass
