#!/usr/bin/env python

import rospy

from chest_mapping import chestMapping

chestControls = chestMapping()

if __name__ == '__main__':
    rospy.init_node("gopher_keyboard_control", anonymous=True)

    while not rospy.is_shutdown():
        pass
