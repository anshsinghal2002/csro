#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer

if  __name__ == '__main__':
    rospy.wait_for_service('register_player')
    register_player = rospy.ServiceProxy('register_player', RegisterPlayer)

    try:
        resp1 = register_player("test_id", "test_color")
        print(resp1)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
