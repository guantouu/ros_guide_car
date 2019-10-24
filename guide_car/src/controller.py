#!/usr/bin/env python3

import rospy
import threading, queue
from std_msgs.msg import String
import sys, select, termios, tty

def getKey(q_input):
    while True:
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        q_input.put(key)
        if key == '\x03':
            break
        

def talker(q_input):
    try:
        pub = rospy.Publisher('keyboard', String, queue_size=1)
        key = 'wait'
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if q_input.qsize() > 0:
                key = q_input.get()
                pub.publish(key)
            else:
                pub.publish(key)
            if key == '\x03':
                break
            rate.sleep()
 
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    q_input = queue.Queue(maxsize=1)
    t_getkey = threading.Thread(target=getKey, args=(q_input,))
    t_talker = threading.Thread(target=talker, args=(q_input,))
    rospy.init_node('keyboard_input')
    try:
        t_getkey.start()
        t_talker.start()
        
    except KeyboardInterrupt as e:
        print(e)
    
