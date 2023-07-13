#!/usr/bin/env python

import rospy
import math
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *

import key_state_machine as ksm

class armMapping:
    def __init__(self, arm_name, rot_x, rot_y, rot_z):
        # Variables
        self.name = arm_name
        self.vel_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lin_vel = 0.25
        self.rot_vel = 0.5
        self.gripper_vel = 0.5
        self.rot_x = rot_x
        self.rot_y = rot_y
        self.rot_z = rot_z

        # Init keys
        # Linear
        # Fast motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', on_press=self.set_vel, on_release=self.reset_vel, index=0, value=self.lin_vel)
        ksm.keyStateMachine(parent_name=self.name, key_name='s', on_press=self.set_vel, on_release=self.reset_vel, index=0, value=-1*self.lin_vel)
        ksm.keyStateMachine(parent_name=self.name, key_name='a', on_press=self.set_vel, on_release=self.reset_vel, index=1, value=self.lin_vel)
        ksm.keyStateMachine(parent_name=self.name, key_name='d', on_press=self.set_vel, on_release=self.reset_vel, index=1, value=-1*self.lin_vel)
        ksm.keyStateMachine(parent_name=self.name, key_name='q', on_press=self.set_vel, on_release=self.reset_vel, index=2, value=self.lin_vel)
        ksm.keyStateMachine(parent_name=self.name, key_name='e', on_press=self.set_vel, on_release=self.reset_vel, index=2, value=-1*self.lin_vel)
        
        # Slow motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=0, value=self.lin_vel*0.25)
        ksm.keyStateMachine(parent_name=self.name, key_name='s', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=0, value=-1*self.lin_vel*0.25)
        ksm.keyStateMachine(parent_name=self.name, key_name='a', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=1, value=self.lin_vel*0.25)
        ksm.keyStateMachine(parent_name=self.name, key_name='d', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=1, value=-1*self.lin_vel*0.25)
        ksm.keyStateMachine(parent_name=self.name, key_name='q', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=2, value=self.lin_vel*0.25)
        ksm.keyStateMachine(parent_name=self.name, key_name='e', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=2, value=-1*self.lin_vel*0.25)

        # Angular
        
        # TODO: Remove if statement
        if self.name == "/right":
            # Fast motion
            ksm.keyStateMachine(parent_name=self.name, key_name='l', on_press=self.set_vel, on_release=self.reset_vel, index=3, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='j', on_press=self.set_vel, on_release=self.reset_vel, index=3, value=-1*self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='k', on_press=self.set_vel, on_release=self.reset_vel, index=4, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='i', on_press=self.set_vel, on_release=self.reset_vel, index=4, value=-1*self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='o', on_press=self.set_vel, on_release=self.reset_vel, index=5, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='u', on_press=self.set_vel, on_release=self.reset_vel, index=5, value=-1*self.rot_vel)
            
            # Slow motion
            ksm.keyStateMachine(parent_name=self.name, key_name='l', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=3, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='j', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=3, value=-1*self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='k', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=4, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='i', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=4, value=-1*self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='o', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=5, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='u', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=5, value=-1*self.rot_vel*0.25)

        elif self.name == "/left":
            # Fast motion
            ksm.keyStateMachine(parent_name=self.name, key_name='j', on_press=self.set_vel, on_release=self.reset_vel, index=3, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='l', on_press=self.set_vel, on_release=self.reset_vel, index=3, value=-1*self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='i', on_press=self.set_vel, on_release=self.reset_vel, index=4, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='k', on_press=self.set_vel, on_release=self.reset_vel, index=4, value=-1*self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='u', on_press=self.set_vel, on_release=self.reset_vel, index=5, value=self.rot_vel)
            ksm.keyStateMachine(parent_name=self.name, key_name='o', on_press=self.set_vel, on_release=self.reset_vel, index=5, value=-1*self.rot_vel)
            
            # Slow motion
            ksm.keyStateMachine(parent_name=self.name, key_name='j', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=3, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='l', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=3, value=-1*self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='i', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=4, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='k', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=4, value=-1*self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='u', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=5, value=self.rot_vel*0.25)
            ksm.keyStateMachine(parent_name=self.name, key_name='o', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, index=5, value=-1*self.rot_vel*0.25)


        # Special action
        ksm.keyStateMachine(parent_name=self.name, key_name='2', on_press=self.gripper_move, on_release=self.gripper_stop, mode=2, value=self.gripper_vel)   # Close gripper
        ksm.keyStateMachine(parent_name=self.name, key_name='1', on_press=self.gripper_move, on_release=self.gripper_stop, mode=2, value=-1*self.gripper_vel)   # Open gripper
       
        # TODO: Predefined positions
        # ksm.keyStateMachine(parent_name=self.name, key_name='f1', key_modifier="shift")
        # ksm.keyStateMachine(parent_name=self.name, key_name='f2', key_modifier="shift")
        # ksm.keyStateMachine(parent_name=self.name, key_name='f3', key_modifier="shift")

        # Init topics and services
        self.cart_vel_pub = rospy.Publisher(self.name + '/in/cartesian_velocity', TwistCommand, queue_size=1)
        self.gripper_command_srv = rospy.ServiceProxy(self.name + '/base/send_gripper_command', SendGripperCommand)


    def Rx(self, theta):
        theta = math.radians(theta)
        return np.matrix([[1, 0, 0],
                        [0, math.cos(theta),-math.sin(theta)],
                        [0, math.sin(theta), math.cos(theta)]])


    def Ry(self, theta):
        theta = math.radians(theta)
        return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                        [0, 1, 0],
                        [-math.sin(theta), 0, math.cos(theta)]])


    def Rz(self, theta):
        theta = math.radians(theta)
        return np.matrix([[math.cos(theta), -math.sin(theta), 0],
                        [math.sin(theta), math.cos(theta) , 0],
                    [0, 0, 1 ]])


    def set_vel(self, **kwargs):
        self.vel_array[kwargs['index']] = kwargs['value']

        # print("press", self.vel_array)
        self.pub_vel(self.vel_array)       


    def reset_vel(self, **kwargs):
        self.vel_array[kwargs['index']] = 0.0   
        
        # print("release", self.vel_array)
        self.pub_vel(self.vel_array)
        


    def pub_vel(self, array):
        # Remove reference
        array = array.copy()
        lin_array = np.array([array[0], array[1], array[2]])
        rot_array = np.array([array[3], array[4], 0.0])

        # Rotate coordinate system
        R_forw = self.Rx(self.rot_x) * self.Ry(self.rot_y) * self.Rz(self.rot_z)

        lin_array = np.round(np.matmul(R_forw, lin_array), 2)
        rot_array = np.round(np.matmul(R_forw, rot_array), 2)

        # Update variables
        array[0] = lin_array[0][0]
        array[1] = lin_array[0][1]
        array[2] = lin_array[0][2]
        array[3] = rot_array[0][0]
        array[4] = rot_array[0][1]

        # Form a new velocity message
        self.msg = TwistCommand()

        self.twist_msg = Twist()

        self.twist_msg.linear_x = array[0]
        self.twist_msg.linear_y = array[1]
        self.twist_msg.linear_z = array[2]
        
        self.twist_msg.angular_x = array[3]
        self.twist_msg.angular_y = array[4]
        self.twist_msg.angular_z = array[5]

        self.msg.reference_frame = 0
        self.msg.twist = self.twist_msg
        self.msg.duration = 0

        # Publish a message
        self.cart_vel_pub.publish(self.msg)
        # print("message", array)
        # print()


    def gripper_move(self, **kwargs):
        self.gripper_control(kwargs['mode'], kwargs['value'])


    def gripper_stop(self, **kwargs):
        self.gripper_control(kwargs['mode'], 0.0)


    # Gripper control: mode=1 - force, mode=2 - velocity, mode=3 - position
    def gripper_control(self, mode, value):
        # rospy.wait_for_service('/my_gen3/base/send_gripper_command')

        self.mode = mode
        self.value = value

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Finger.msg
        self.finger = Finger()
        self.finger.finger_identifier = 0
        self.finger.value = self.value

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Gripper.msg
        self.gripper = Gripper()
        self.gripper.finger.append(self.finger)

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/GripperCommand.msg
        self.gripper_command = SendGripperCommand()
        self.gripper_command.mode = self.mode
        self.gripper_command.duration = 0
        self.gripper_command.gripper = self.gripper

        self.gripper_command_srv(self.gripper_command)

