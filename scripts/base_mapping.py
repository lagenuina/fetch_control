#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from math import pi

import key_state_machine as ksm

class baseMapping():
    def __init__(self, name):

        # Variables
        self.name = name
        
        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        self.current_lin_accel = 0.0
        self.current_rot_accel = 0.0
        
        self.current_lin_jerk = 0.0
        self.current_rot_jerk = 0.0

        self.max_lin_vel = 0.5 # meters
        self.min_lin_vel = -0.2 # meters
        self.max_rot_vel = pi/2.0 # rads per sec (90 degrees per sec)
        self.min_rot_vel = -pi/2.0

        self.max_lin_accel = self.calc_accel(vel = self.max_lin_vel, time = 1.0)
        self.min_lin_accel = -self.max_lin_accel  # this should be deceleration
        self.max_rot_accel = self.calc_accel(vel = self.max_rot_vel, time = 1.0)
        self.min_rot_accel = -self.max_rot_accel  # this should be deceleration

        self.smooth_out_lin_vel = 0.1
        self.smooth_out_rot_vel = pi/4.0

        self.pos_lin_jerk = self.calc_jerk( vel_change = self.smooth_out_lin_vel, 
                                            accel = self.max_lin_accel) 
        self.neg_lin_jerk = self.calc_jerk( vel_change = -self.smooth_out_lin_vel, 
                                            accel = self.min_lin_accel) 

        self.pos_rot_jerk = self.calc_jerk( vel_change = self.smooth_out_rot_vel, 
                                            accel = self.max_rot_accel) 
        self.neg_rot_jerk = self.calc_jerk( vel_change = -self.smooth_out_rot_vel, 
                                            accel = self.min_rot_accel) 
           
        self.previous_time = rospy.get_time()

        # Init keys
        # Fast motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', on_press=self.set_vel, on_release=self.reset_vel, type="lin")
        ksm.keyStateMachine(parent_name=self.name, key_name='s', on_press=self.set_vel, on_release=self.reset_vel, type="lin")
        ksm.keyStateMachine(parent_name=self.name, key_name='a', on_press=self.set_vel, on_release=self.reset_vel, type="rot")
        ksm.keyStateMachine(parent_name=self.name, key_name='d', on_press=self.set_vel, on_release=self.reset_vel, type="rot")

        # Slow motion
        ksm.keyStateMachine(parent_name=self.name, key_name='w', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, type="lin")
        ksm.keyStateMachine(parent_name=self.name, key_name='s', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, type="lin")
        ksm.keyStateMachine(parent_name=self.name, key_name='a', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, type="rot")
        ksm.keyStateMachine(parent_name=self.name, key_name='d', key_modifier="shift", on_press=self.set_vel, on_release=self.reset_vel, type="rot")

        # Init topics and services
        self.twist_pub = rospy.Publisher('base_controller/command', Twist, queue_size=1)


    def set_vel(self, **kwargs):
        if kwargs['type'] == "lin":
            self.current_lin_accel = self.max_lin_accel

        elif kwargs['type'] == "rot":
            self.current_rot_accel = self.max_rot_accel

        try:
            accel = Float32()
            accel.data = self.current_lin_accel
            self.accel_pub.publish(accel)

        except Exception as e:
            print(e)

        self.update_target_vel()
        self.pub_target_vel()   


    def reset_vel(self, **kwargs):
        if kwargs['type'] == "lin":
            self.choose_lin_accel_toward_stopping()

        elif kwargs['type'] == "rot":
            self.choose_rot_accel_toward_stopping()

        self.update_target_vel()
        self.pub_target_vel()  


    def zero_vel(self):
        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0


    def zero_accel(self):
        self.current_lin_accel = 0.0
        self.current_rot_accel = 0.0


    def calc_accel(self, vel, time):
        '''
        @params vel [m/sec or rad/sec]
        @params time [sec]

        returns - the acceleration
        '''
        return float(vel)/float(time)


    def key_map_on_release(self, key):
        # TODO Added the response when a button is released
        print(key)


    def calc_jerk(self, vel_change, accel):
        """
        calculates the needed jerk to make the targeted shift in the velocity:
        
        we dont really care how long it will take to reduce the speed to 0. 
        """
        # we will be doing a sodo intergral of the acceleration curve
        # vel_change = 0.5 * accel * time    --> time = vel_change / (0.5 * accel)
        # jerk = accel / time                --> time = accel / jerk
        #                                    --> vel_change / (0.5 * accel) = accel / jerk
        jerk = accel / vel_change * (0.5 * accel)
        return jerk


    def get_time_step_in_sec(self):

        current_time = rospy.get_time()
        duration = current_time - self.previous_time

        self.previous_time = current_time

        # print(str(duration))
        return duration


    def pub_vel(self, x_vel, w_vel):
        """
        Send the twist msg to the base
        """
        msg = Twist()
        msg.linear.x = x_vel
        msg.angular.z = w_vel

        self.current_lin_vel = x_vel
        self.current_rot_vel = w_vel
  
        self.twist_pub.publish(msg)


    def pub_target_vel(self):
        self.pub_vel(self.target_lin_vel, self.target_rot_vel)


    def update_target_vel(self):

        # lets consider the idea of changing the acceleration of the robot instead of worrying about other bs.
        time_step = self.get_time_step_in_sec()

        self.target_lin_vel = self.current_lin_vel + self.current_lin_accel*time_step
        self.target_rot_vel = self.current_rot_vel + self.current_rot_accel*time_step

        # I have always set the acceleration to be a non zero value, except for when I want the system to not move.
        # There are definietly better ways to do this but:

        if self.current_lin_accel == 0: # if we are not trying to accelerate
            self.target_lin_vel = 0
        else:
            # constraining the velocity of the mobile base
            if self.target_lin_vel > self.max_lin_vel: self.target_lin_vel = self.max_lin_vel
            elif self.target_lin_vel < self.min_lin_vel: self.target_lin_vel = self.min_lin_vel

        if self.current_rot_accel == 0: # if we are not trying to accelerate
            self.target_rot_vel = 0
        else:
            # constraining the velocity of the mobile base
            if self.target_rot_vel > self.max_rot_vel: self.target_rot_vel = self.max_rot_vel
            elif self.target_rot_vel < self.min_rot_vel: self.target_rot_vel = self.min_rot_vel

        # print(self.target_lin_vel, self.target_rot_vel)


    def hard_stop(self):
        self.pub_vel(0.0, 0.0)
        # Reseting all the current and target accelerations and velocities
        self.zero_accel()
        self.zero_vel()


    def choose_target_accel_toward_stopping(self, error_around_zero, smooth_out_vel_bound, target_vel, accel, deaccel, current_accel, pos_jerk, neg_jerk):
        # choosing the acceleration to slow down the base

        # i need some velocity to start smoothing at the vel curve

        if target_vel > smooth_out_vel_bound: return deaccel
        elif target_vel > error_around_zero: return 0.5*deaccel
        elif target_vel < -smooth_out_vel_bound: return accel
        elif target_vel < -error_around_zero: return 0.5*accel
        else: return 0.0


    def choose_target_accel_toward_stopping_old_imp(self, error_around_zero, smooth_out_vel_bound, target_vel, accel, deaccel, current_accel, pos_jerk, neg_jerk):
        # choosing the acceleration to slow down the base

        # i need some velocity to start smoothing at the vel curve

        if target_vel > error_around_zero: return deaccel
        
        elif target_vel < -error_around_zero: return accel
        
        else: return 0.0


    def update_accel_using_jerk(self, current_accel, jerk):
        time_step = self.get_time_step_in_sec()
        return current_accel + jerk*time_step


    def choose_lin_accel_toward_stopping(self):
        # we should choose a new acceleration in the attempot to slow down the base
        self.current_lin_accel = self.choose_target_accel_toward_stopping(   error_around_zero = 0.01,
                                                                        smooth_out_vel_bound = self.smooth_out_lin_vel, 
                                                                        target_vel = self.target_lin_vel,
                                                                        accel = self.max_lin_accel,
                                                                        deaccel = self.min_lin_accel,
                                                                        current_accel = self.current_lin_accel,
                                                                        pos_jerk = self.pos_lin_jerk,
                                                                        neg_jerk= self.neg_lin_jerk)
                                                        

    def choose_rot_accel_toward_stopping(self):
        # we should choose a new acceleration in the attempot to slow down the base
        self.current_rot_accel = self.choose_target_accel_toward_stopping_old_imp(   error_around_zero = 0.05,
                                                                                smooth_out_vel_bound = self.smooth_out_rot_vel, 
                                                                                target_vel = self.target_rot_vel,
                                                                                accel = self.max_rot_accel,
                                                                                deaccel = self.min_rot_accel,
                                                                                current_accel = self.current_rot_accel,
                                                                                pos_jerk = self.pos_rot_jerk,
                                                                                neg_jerk= self.neg_rot_jerk)