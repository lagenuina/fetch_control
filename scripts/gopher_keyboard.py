#!/usr/bin/env python

import rospy

import key_listener as kl
from chest_mapping import chestMapping
from arm_mapping import armMapping

import key_state_machine as ksm

def on_shutdown():
    # Stop the chest motion
    chestControls.stop_srv()

def change_module(**kwargs):
    try:
        kl.target_name = kwargs["module_name"]
        print(kl.target_name)

    except Exception as e:
        print(e)

if __name__ == '__main__':

    
    rightArmControls = armMapping("/right", 0, -45, 90)
    leftArmControls = armMapping("/left", 0, -45, -90)
    chestControls = chestMapping("/chest")

    ksm.keyStateMachine(key_name='up', on_press=change_module, module_name="/chest")
    ksm.keyStateMachine(key_name='down', on_press=change_module, module_name="/base")
    ksm.keyStateMachine(key_name='left', on_press=change_module, module_name="/left")
    ksm.keyStateMachine(key_name='right', on_press=change_module, module_name="/right")

    kl.keyboard_init()

    rospy.init_node("gopher_keyboard_control", anonymous=True)


    # for keyObject in ksm.keyStateMachine._registry:

    #     print(keyObject.key_name, keyObject.parent_name)

    #     # https://stackoverflow.com/questions/15924772/how-to-get-name-of-class-if-i-know-only-a-bound-method-in-python
    #     # print(keyObject.key_name, keyObject.on_press.__self__.__class__.__name__)

    while not rospy.is_shutdown():
        pass

    # Safe function if the node dies: only service calls or parameter setting (NO PUBLISHING)
    rospy.on_shutdown(on_shutdown)