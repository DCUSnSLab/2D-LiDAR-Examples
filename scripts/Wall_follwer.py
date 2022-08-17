# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

speed_pub = rospy.Publisher('/vesc/commands/motor/speed', Float64, queue_size=1)
position_pub = rospy.Publisher('/vesc/commands/servo/position', Float64, queue_size=1)

state_ = 0

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

regions = None

def callback(data):
    global state_, regions
    regions = {
        'right': min(data.ranges[60:89]),
        'fright': min(data.ranges[30:59]),
        'front': min(min(data.ranges[0:29]), min(data.ranges[330:359])),
        'fleft': min(data.ranges[300:329]),
        'left': min(data.ranges[270:299]),
    }

    distance = 2.0

    take_action(distance)

    print("state_", state_)

    if state_ == 0:
        find_wall()
    elif state_ == 1:
        turn_left()
    elif state_ == 2:
        follow_the_wall()
        pass
    else:
        rospy.logerr('Unknown state!')

def change_state(state):
    global state_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action(distance):
    global regions
    d = distance

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    speed = 7000
    position = 0.5304 - 0.4

    speed_pub.publish(speed)
    position_pub.publish(position)

def turn_left():
    position = 0.5304 + 0.4

    position_pub.publish(position)

def follow_the_wall():
    speed = 7000
    position = 0.5304

    speed_pub.publish(speed)
    position_pub.publish(position)

if __name__ == '__main__':
    rospy.init_node("WallFollwer")
    sub = rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
