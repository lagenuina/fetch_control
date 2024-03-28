#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import math


class baseMapping():

    def __init__(self):

        self.button_pressed = None
        self.isPressed = None
        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        self.max_lin_vel = 0.5  # meters

        self.max_rot_vel = math.radians(60)

        self.lin_accel = 0.5 / 1.0  # m/s^2
        self.rot_accel = math.radians(600)  # rad/s^2

        self.previous_time = rospy.get_time()

        # Init topics and services
        self.twist_pub = rospy.Publisher(
            'base_controller/command', Twist, queue_size=1
        )

        rospy.Subscriber('button_pressed', Int32, self.buttons_feedback)

    def buttons_feedback(self, msg):

        self.button_pressed = msg.data

    def get_time_step_in_sec(self):

        current_time = rospy.get_time()
        duration = current_time - self.previous_time

        self.previous_time = current_time

        return duration

    def set_velocities(self):

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        if self.button_pressed == 1:
            self.target_lin_vel = self.max_lin_vel

        elif self.button_pressed == 2:
            self.target_lin_vel = -1 * self.max_lin_vel * 0.5

        elif self.button_pressed == 3:
            self.target_rot_vel = -self.max_rot_vel

        elif self.button_pressed == 4:
            self.target_rot_vel = self.max_rot_vel

    def pub_vel(self):

        self.set_velocities()
        """
        Send the twist msg to the base
        """
        # Calculate new velocity values
        self.current_lin_vel = self.update_vel(
            self.target_lin_vel, self.current_lin_vel, self.lin_accel
        )
        self.current_rot_vel = self.update_vel(
            self.target_rot_vel, self.current_rot_vel, self.rot_accel
        )

        # Form a message
        msg = Twist()
        msg.linear.x = self.current_lin_vel
        msg.angular.z = self.current_rot_vel

        # Publish a message
        self.twist_pub.publish(msg)

    def update_vel(self, target_vel, current_vel, accel):
        # Get updated time step
        time_step = self.get_time_step_in_sec()

        # Forward motion
        if target_vel > 0:
            # Target velocity reached
            if current_vel >= target_vel:
                current_vel = target_vel

            # Acceleration
            else:
                current_vel = current_vel + accel * time_step

        # Backward motion
        elif target_vel < 0:
            # Target velocity reached
            if current_vel <= target_vel:
                current_vel = target_vel

            # Acceleration
            else:
                current_vel = current_vel - accel * time_step

        # Stopping
        elif target_vel == 0:
            # Target velocity reached
            if abs(current_vel - target_vel) < 0.05:
                current_vel = 0.0

            # Decceleration
            else:
                if current_vel > 0:
                    current_vel = current_vel - accel * time_step

                elif current_vel < 0:
                    current_vel = current_vel + accel * time_step

        return current_vel


if __name__ == '__main__':

    rospy.init_node("base_manual_control", anonymous=True)

    base_control = baseMapping()

    while not rospy.is_shutdown():
        # Update mobile base velocities and publish
        base_control.pub_vel()
