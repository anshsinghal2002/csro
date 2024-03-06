#!/usr/bin/env python3

import rospy
from csro.srv import RegisterPlayer

# Players list
# Stores all RegisterPlayer objects
players = []

def register_player(req):
    players.append(req)
    print(req)
    return 1

if  __name__ == '__main__':
    rospy.init_node('csro_core', anonymous=True)
    s = rospy.Service('register_player', RegisterPlayer, register_player)

    try:
        print("csro_core started")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

