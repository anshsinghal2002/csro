#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty

rospy.init_node('game_starter')
pub = rospy.Publisher('/start_game', Empty, queue_size=10)

while True:
    input('Press enter to start')
    pub.publish(Empty())
    print('Published')