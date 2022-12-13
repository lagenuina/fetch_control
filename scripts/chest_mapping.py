#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gopher_ros_clearcore.msg import *
from gopher_ros_clearcore.srv import *

import key_state_machine as ksm

class chestMapping():
    def __init__(self):
        # Variables
        self.high_velocity = 1.0
        self.low_velocity = 0.5
        self.high_pos = 440
        self.mid_pos = 220
        self.low_pos = 0

        # Init keys
        # Fast motion
        ksm.keyStateMachine(key_name='w', on_hold=self.move_up, z_vel = self.high_velocity)
        ksm.keyStateMachine(key_name='s', on_hold=self.move_down, z_vel = self.high_velocity)

        # Slow motion
        ksm.keyStateMachine(key_name='w', key_modifier="shift", on_hold=self.move_up, z_vel=self.low_velocity)
        ksm.keyStateMachine(key_name='s', key_modifier="shift", on_hold=self.move_down, z_vel=self.low_velocity)

        ksm.keyStateMachine(key_name='h', on_hold=self.homing)
        ksm.keyStateMachine(key_name='space', on_hold=self.stop)

        # Predefined positions
        ksm.keyStateMachine(key_name='f1', key_modifier="shift", on_hold=self.low_position, low_pos=self.low_pos, vel=self.low_velocity)
        ksm.keyStateMachine(key_name='f2', key_modifier="shift", on_hold=self.middle_position, low_pos=self.mid_pos, vel=self.low_velocity)
        ksm.keyStateMachine(key_name='f3', key_modifier="shift", on_hold=self.high_position, low_pos=self.high_pos, vel=self.low_velocity)

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
    def pub_vel(self, z_vel):
        msg = Twist()
        msg.linear.z = z_vel
        self.z_axis_twist_pub.publish(msg)

    def move_up(self, **kwargs):
        self.pub_vel(kwargs['z_vel'])

    def move_down(self, **kwargs):
        self.pub_vel(-1 * kwargs['z_vel'])

    def stop(self, **kwargs):
        self.chest_stop_srv(1)

    def homing(self, **kwargs):
        self.homing_srv(1)

    def low_position(self, **kwargs):
        self.abspos_srv(position=kwargs['low_pos'], velocity=kwargs['vel'])
    
    def middle_position(self, **kwargs):
        self.abspos_srv(position=kwargs['mid_pos'], velocity=kwargs['vel'])

    def high_position(self, **kwargs):
        self.abspos_srv(position=kwargs['high_pos'], velocity=kwargs['vel'])

