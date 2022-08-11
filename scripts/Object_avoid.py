# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class ObjectAvoid:
    def __init__(self):
        self.sub = rospy.Subscriber("/lidar2D", LaserScan, self.callback)

        self.speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

    def callback(self, data):
        """
        입력되는 Lidar 데이터의 경우 0도를 기준으로 전방에서 시계방향으로 360도 각 방향을 가리킵니다.
        따라서 ranges의 0 ~ 30, 330 ~ 359 영역을 전방으로 정의하여 간단한 장애물 회피 코드를 작성했습니다.
        """
        if min(data.ranges[0:30]) < 5 or min(data.ranges[330:359]) < 5:
            """
            전방 0 ~ 30도 또는 330 ~ 359도에 장애물이 있는 경우,
            """
            if min(data.ranges[0:30]) < min(data.ranges[330:359]):
                """
                왼쪽에 장애물이 있는 경우 우회전
                """
                self.speed.publish(7000)
                self.position.publish(0.5304 + 0.2)
            else:
                """
                오른쪽에 장애물이 있는 경우 좌회전한다.
                """
                self.speed.publish(7000)
                self.position.publish(0.5304 - 0.2)
        else:
            self.speed.publish(10000)
            self.position.publish(0.5304)

if __name__ == '__main__':
    try:
        rospy.init_node("ObjectAvoid")
        run = ObjectAvoid()
        rospy.spin()
    except:
        print("Error occured.")