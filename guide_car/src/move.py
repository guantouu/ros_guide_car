import rospy
import sys, select, termios, tty
from adafruit_motorkit import MotorKit
from std_msgs.msg import String
from ultra_wideband.srv import pozyx_data

class Motor():
    def __init__(self):
        self.kit = MotorKit()
        self.srv = pozyx_data()

    def Forward(self, M2=0.8, M3=0.8):
        self.kit.motor1.throttle = 0.7
        self.kit.motor2.throttle = M2
        self.kit.motor3.throttle = M3
        self.kit.motor4.throttle = 0.7

    def Backward(self, M1=-0.8, M4=-0.8):
        self.kit.motor1.throttle = M1
        self.kit.motor2.throttle = -0.7
        self.kit.motor3.throttle = -0.7
        self.kit.motor4.throttle = M4

    def Right(self, M3=-0.8, M4=0.8):
        self.kit.motor1.throttle = -0.7
        self.kit.motor2.throttle = 0.7
        self.kit.motor3.throttle = M3
        self.kit.motor4.throttle = M4

    def Left(self, M1=0.8, M2=-0.8):
        self.kit.motor1.throttle = M1
        self.kit.motor2.throttle = M2
        self.kit.motor3.throttle = 0.7
        self.kit.motor4.throttle = -0.7

    def Stop(self):
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0
        self.kit.motor3.throttle = 0
        self.kit.motor4.throttle = 0

    def move(self, data):
        client = rospy.ServiceProxy('angle', self.srv)
        angle = int(client(True).angle)
        key=str(data.data)
        print(angle)
        if key == 'w':
            #print('forward')
            if angle <= 90:
                self.Forward(M3=1)
            elif angle > 90:
                self.Forward(M2=1)

        elif key == 's':
            #print('backward')
            if angle <= 90:
                self.Backward(M1=-1)
            elif angle > 90:
                self.Backward(M4=-1)

        elif key == 'a':
            #print('left')
            if angle > 90:
                self.Left(M1=1)
            elif angle <= 90:
                self.Left(M2=-1)

        elif key == 'd':
            #print('right')
            if angle <= 90:
                self.Right(M4=1)
            elif angle > 90:
                self.Right(M3=-1)
        else:
            #print('stop')
            self.Stop()
    
def listener():
    car = Motor()
    try:
        rospy.init_node('Motor', anonymous=True)
        rospy.Subscriber('keyboard', String, car.move)
        rospy.spin()
    except rospy.ServiceException as e:
        car.Stop()
        print('stop')
    

if __name__ == "__main__":
    listener()
