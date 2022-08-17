# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

speed_pub = rospy.Publisher('(시뮬레이터의 speed Subscriber와 동일하게 지정)', Float64, queue_size=1)
position_pub = rospy.Publisher('(시뮬레이터의 position Subscriber와 동일하게 지정)', Float64, queue_size=1)

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

    '''
    각 상태에 따라 수행할 동작을 아래 조건문을 통해 판단한다.
    '''
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
        """
        전방 영역의 값, 좌전방 영역의 값, 우전방 영역의 값이 모두 distance보다 큰 경우,
        차량이 오른쪽으로 이동하도록 상태를 갱신한다.
        """
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        """
        전방 영역의 값이 distance보다 작고,
        좌전방 영역의 값이 distance보다 크고,
        우전방 영역의 값이 distance보다 큰 경우,
        차량이 왼쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        """
        전방 영역의 값이 distance보다 크고,
        좌전방 영역의 값이 distance보다 크고,
        우전방 영역의 값이 distance보다 작은 경우,
        차량이 직진하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        """
        전방 영역의 값이 distance보다 크고,
        좌전방 영역의 값이 distance보다 작고,
        우전방 영역의 값이 distance보다 큰 경우,
        차량이 오른쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        """
        전방 영역의 값이 distance보다 작고,
        좌전방 영역의 값이 distance보다 크고,
        우전방 영역의 값이 distance보다 작은 경우,
        차량이 왼쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        """
        전방 영역의 값이 distance보다 작고,
        좌전방 영역의 값이 distance보다 작고,
        우전방 영역의 값이 distance보다 큰 경우,
        차량이 왼쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        """
        전방 영역의 값, 좌전방 영역의 값, 우전방 영역의 값이 모두 distance보다 작은 경우,
        차량이 왼쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        """
        전방 영역의 값이 distance보다 크고,
        좌전방 영역의 값이 distance보다 작고,
        우전방 영역의 값이 distance보다 작은 경우,
        차량이 오른쪽으로 이동하도록 change_state() 함수를 통해 상태를 갱신한다.
        """
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

'''
find_wall(), turn_left(), follow_the_wall()
함수들을 통해 각 상태별 차량의 행동을 수행한다.
'''

def find_wall():
    '''
    Publisher speed, position을 사용하여 차량이 오른쪽으로 이동하도록 임의의 값을 publish한다.
    '''

def turn_left():
    '''
    Publisher speed, position을 사용하여 차량이 왼쪽으로 이동하도록 임의의 값을 publish한다.
    '''

def follow_the_wall():
    '''
    Publisher speed, position을 사용하여 차량이 전방으로 이동하도록 임의의 값을 publish한다.
    '''

if __name__ == '__main__':
    rospy.init_node("WallFollwer")
    sub = rospy.Subscriber("(시뮬레이터 내 LiDAR 센서의 Topic)", LaserScan, callback)
    rospy.spin()
