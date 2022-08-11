# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

class WallFollower:
    def __init__(self):
        print("__init__ called.")
        self.sub = rospy.Subscriber("/lidar2D", LaserScan, self.callback)

        self.speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        self.state_ = 0

        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

        self.regions = None

    def callback(self, data):
        self.regions = {
            'right': min(data.ranges[60:89]),
            'fright': min(data.ranges[30:59]),
            'front': min(min(data.ranges[0:29]), min(data.ranges[330:359])),
            'fleft': min(data.ranges[300:329]),
            'left': min(data.ranges[270:299]),
        }

        distance = 1.0

        self.take_action(distance)

        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.turn_left()
        elif self.state_ == 2:
            self.follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

    def change_state(self, state):
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def take_action(self, distance):
        d = distance

        if self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] > d:
            # state_description = 'case 1 - nothing'
            self.change_state(0)
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] > d:
            # state_description = 'case 2 - front'
            self.change_state(1)
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] < d:
            # state_description = 'case 3 - fright'
            self.change_state(2)
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] > d:
            # state_description = 'case 4 - fleft'
            self.change_state(0)
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] < d:
            # state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] > d:
            # state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] < d:
            # state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] < d:
            # state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(self.regions)

    def find_wall(self):
        speed = 7000
        position = 0.5304 - 0.4

        self.speed.publish(speed)
        self.position.publish(position)

    def turn_left(self):
        position = 0.5304 + 0.4

        self.position.publish(position)

    def follow_the_wall(self):
        speed = 7000
        position = 0.5304

        self.speed.publish(speed)
        self.position.publish(position)

if __name__ == '__main__':
    rospy.init_node("WallFollwer")
    run = WallFollower()
    rospy.spin()
