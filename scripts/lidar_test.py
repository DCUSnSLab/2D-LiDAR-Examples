# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
def callback(data):	
    """
    Lidar 데이터는 0도를 기준으로 전방에서 시계방향으로 360도 각도의 거리데이터가 수신됨
    1도마다 거리데이터(mm) 수신
    아래 print 함수는 테스트 후 제거하고 진행
    """
    print('0 deg dis : ',data.ranges[0]) #각도가 0인 거리데이터 출력

    print('0~5 deg dis : ',data.ranges[0:5]) #0~30 각도의 데이터 출력

    print('min of 0~5 deg : ',min(data.ranges[0:30])) #0~30 각도중 가장 작은 값 출력
		
if __name__ == '__main__':
    try:
        rospy.init_node("ObjectAvoid")
        sub = rospy.Subscriber("/lidar2D", LaserScan, callback)
        rospy.spin()
    except:
        print("Error occured.")

