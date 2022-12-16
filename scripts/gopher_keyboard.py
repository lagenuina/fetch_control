#!/usr/bin/env python

import rospy

import key_listener as kl
from chest_mapping import chestMapping
from arm_mapping import armMapping


def on_shutdown():
    # Stop the chest motion
    chestControls.stop_srv()

if __name__ == '__main__':

    
    armControls = armMapping("/my_gen3", 0, -45, 90)
    chestControls = chestMapping()

    kl.keyboard_init()

    rospy.init_node("gopher_keyboard_control", anonymous=True)

    while not rospy.is_shutdown():
        pass

    # Safe function if the node dies: only service calls or parameter setting (NO PUBLISHING)
    rospy.on_shutdown(on_shutdown)