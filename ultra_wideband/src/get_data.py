#!/usr/bin/env python3 

from pypozyx import (PozyxConstants, PozyxSerial, Coordinates, 
                    EulerAngles, get_first_pozyx_serial_port)
import rospy
from geometry_msgs.msg import Point32
from ultra_wideband.srv import pozyx_data

class DataSet():
    def __init__(self):
        self.pozyx = PozyxSerial(get_first_pozyx_serial_port())
        self.direct = EulerAngles()
        self.position = Coordinates()

    def get_angle(self, req):
        if req.sigin == True:
            self.pozyx.getEulerAngles_deg(self.direct, remote_id=None)
            return int(self.direct.heading)

def get_data():
    dataset = DataSet()
    srv = pozyx_data()
    rospy.init_node('Get_data', anonymous=True)
    rospy.Service('angle', srv, dataset.get_angle)
    rospy.spin()

if __name__ == "__main__":
    try:
        get_data()
    except rospy.ROSInterruptException:
        pass
