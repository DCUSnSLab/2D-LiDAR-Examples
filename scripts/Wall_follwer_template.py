# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

def degTorad(deg):
	rad_diff = 0.5304
	rad = deg * (3.14/180)
	return rad + rad_diff
    
def callback(data):
    '''
    Publisher speed,
    '''
    speed = rospy.Publisher('(시뮬레이터의 speed Subscriber와 동일하게 지정)', Float64, queue_size=1)
    position = rospy.Publisher('(시뮬레이터의 position Subscriber와 동일하게 지정)', Float64, queue_size=1)


if __name__ == '__main__':
    try:
        rospy.init_node("ObjectAvoid")
        sub = rospy.Subscriber("(시뮬레이터 내 LiDAR 센서의 Topic)", LaserScan, callback)
        rospy.spin()
    except:
        print("Error occured.")