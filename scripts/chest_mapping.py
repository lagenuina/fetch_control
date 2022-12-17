#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gopher_ros_clearcore.msg import *
from gopher_ros_clearcore.srv import *

import key_state_machine as ksm

class chestMapping():
    def __init__(self, name):
        # Variables
        self.name = name
        self.high_velocity = 1.0
        self.low_velocity = 0.5
        self.high_pos = 440
        self.mid_pos = 220
        self.low_pos = 0

        # Init keys
        # Fast motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', on_press=self.move_vel, on_release=self.stop, vel = self.high_velocity)
        ksm.keyStateMachine(parent_name=self.name, key_name='s', on_press=self.move_vel, on_release=self.stop, vel = -1*self.high_velocity)

        # Slow motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', key_modifier="shift", on_press=self.move_vel, on_release=self.stop, vel=self.low_velocity)
        ksm.keyStateMachine(parent_name=self.name, key_name='s', key_modifier="shift", on_press=self.move_vel, on_release=self.stop, vel=-1*self.low_velocity)

        # Special actions
        ksm.keyStateMachine(parent_name=self.name, key_name='h', on_press=self.homing)
        ksm.keyStateMachine(parent_name=self.name, key_name='space', on_press=self.stop_srv)

        # Predefined positions
        ksm.keyStateMachine(parent_name=self.name, key_name='f1', key_modifier="shift", on_press=self.high_position, pos=self.high_pos, vel=self.low_velocity)
        ksm.keyStateMachine(parent_name=self.name, key_name='f2', key_modifier="shift", on_press=self.middle_position, pos=self.mid_pos, vel=self.low_velocity)
        ksm.keyStateMachine(parent_name=self.name, key_name='f3', key_modifier="shift", on_press=self.low_position, pos=self.low_pos, vel=self.low_velocity)


        # Init topics and services
        self.z_chest_vel_pub = rospy.Publisher('z_chest_vel', Twist, queue_size=1)

        self.chest_stop_srv = rospy.ServiceProxy('z_chest_stop', Stop)
        # self.drive_control_srv = rospy.ServiceProxy('z_chest_drive', DriveControl)
        # self.brake_control_srv = rospy.ServiceProxy('z_chest_brake', BrakeControl)
        # self.debug_control_srv = rospy.ServiceProxy('z_chest_debug', DebugControl)
        # self.logger_control_srv = rospy.ServiceProxy('z_chest_logger', LoggerControl)
        self.homing_srv = rospy.ServiceProxy('z_chest_home', Homing)
        self.abspos_srv = rospy.ServiceProxy('z_chest_abspos', AbsolutePosition)
        # self.relpos_srv = rospy.ServiceProxy('z_chest_relpos', RelativePosition)
        
        
    # TODO: Add responses
    def pub_vel(self, vel):
        msg = Twist()
        msg.linear.z = vel
        self.z_chest_vel_pub.publish(msg)

    def move_vel(self, **kwargs):
        self.pub_vel(kwargs['vel'])

    def stop(self, **kwargs):
        self.pub_vel(0)

    def stop_srv(self, **kwargs):
        self.chest_stop_srv(1)

        # TODO: Service response
        response = True

        return response

    def homing(self, **kwargs):
        self.homing_srv(1)

    def low_position(self, **kwargs):
        self.abspos_srv(position=kwargs['pos'], velocity=kwargs['vel'])
    
    def middle_position(self, **kwargs):
        self.abspos_srv(position=kwargs['pos'], velocity=kwargs['vel'])

    def high_position(self, **kwargs):
        self.abspos_srv(position=kwargs['pos'], velocity=kwargs['vel'])

