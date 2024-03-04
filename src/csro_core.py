#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer

def register_player(req):
    print("Req:", req)
    return 1

if  __name__ == '__main__':
    rospy.init_node('csro_core', anonymous=True)
    s = rospy.Service('register_player', RegisterPlayer, register_player)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

