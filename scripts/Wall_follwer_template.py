# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

def callback(data):
    '''
    Publisher speed,
    '''
    speed = rospy.Publisher('(시뮬레이터의 speed Subscriber와 동일하게 지정)', Float64, queue_size=1)
    position = rospy.Publisher('(시뮬레이터의 position Subscriber와 동일하게 지정)', Float64, queue_size=1)
    """
    LaserScan 메시지 자료형의 data.ranges를 사용하여 회피주행을 진행한다.
    ranges의 값은 총 360개가 들어오며,
    0도의 경우 차량 정면,
    90도의 경우 차량 우측,
    180도는 차량 뒷편,
    270도는 차량 좌측에 해당한다. 
    """

if __name__ == '__main__':
    try:
        rospy.init_node("ObjectAvoid")
        sub = rospy.Subscriber("(시뮬레이터 내 LiDAR 센서의 Topic)", LaserScan, callback)
        rospy.spin()
    except:
        print("Error occured.")