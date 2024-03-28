#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
import math

import key_state_machine as ksm


class baseMapping():

    def __init__(self, name):

        # Variables
        self.name = name

        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        self.max_lin_vel = 0.5  # meters

        self.max_rot_vel = math.radians(60)

        self.lin_accel = 0.5 / 1.0  # m/s^2
        self.rot_accel = math.radians(600)  # rad/s^2

        self.previous_time = rospy.get_time()

        # Init keys
        # Fast motion
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='w',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="lin",
            vel=self.max_lin_vel
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='s',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="lin",
            vel=-1 * self.max_lin_vel * 0.5
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='a',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="rot",
            vel=self.max_rot_vel
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='d',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="rot",
            vel=-1 * self.max_rot_vel
        )

        # Slow motion
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='w',
            key_modifier='shift',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="lin",
            vel=0.15
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='s',
            key_modifier='shift',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="lin",
            vel=-1 * 0.15
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='a',
            key_modifier='shift',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="rot",
            vel=self.max_rot_vel * 0.25
        )
        ksm.keyStateMachine(
            parent_name=self.name,
            key_name='d',
            key_modifier='shift',
            on_press=self.set_vel,
            on_release=self.reset_vel,
            type="rot",
            vel=-1 * self.max_rot_vel * 0.25
        )

        # Init topics and services
        self.twist_pub = rospy.Publisher(
            'base_controller/command', Twist, queue_size=1
        )

    def set_vel(self, **kwargs):
        if kwargs['type'] == "lin":
            self.target_lin_vel = kwargs['vel']

        elif kwargs['type'] == "rot":
            self.target_rot_vel = kwargs['vel']

    def reset_vel(self, **kwargs):
        if kwargs['type'] == "lin":
            self.target_lin_vel = 0.0

        elif kwargs['type'] == "rot":
            self.target_rot_vel = 0.0

    def get_time_step_in_sec(self):

        current_time = rospy.get_time()
        duration = current_time - self.previous_time

        self.previous_time = current_time

        return duration

    def pub_vel(self):
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
        # print(round(self.current_lin_vel, 2), round(self.current_rot_vel, 2))

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
