#!/usr/bin/env python

import rospy

import key_listener as kl
from base_mapping import baseMapping

import key_state_machine as ksm

def change_module(**kwargs):
    try:
        kl.target_name = kwargs["module_name"]
        print(kl.target_name)

    except Exception as e:
        print(e)

if __name__ == '__main__':

    rospy.init_node("gopher_keyboard_control", anonymous=True)   

    baseControls = baseMapping("/base")

    ksm.keyStateMachine(key_name='up', on_press=change_module, module_name="/chest")
    ksm.keyStateMachine(key_name='down', on_press=change_module, module_name="/base")
    ksm.keyStateMachine(key_name='left', on_press=change_module, module_name="/left")
    ksm.keyStateMachine(key_name='right', on_press=change_module, module_name="/right")

    kl.keyboard_init()

    # for keyObject in ksm.keyStateMachine._registry:

    #     print(keyObject.key_name, keyObject.parent_name)

    #     # https://stackoverflow.com/questions/15924772/how-to-get-name-of-class-if-i-know-only-a-bound-method-in-python
    #     # print(keyObject.key_name, keyObject.on_press.__self__.__class__.__name__)

    while not rospy.is_shutdown():
        # Update mobile base velocities and publish
        baseControls.pub_vel()