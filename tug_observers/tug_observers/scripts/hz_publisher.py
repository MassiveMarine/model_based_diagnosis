#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from threading import Thread


class InputThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.setDaemon(True)
        self.rate = 1
        self.changed = False

    def run(self):
        while not rospy.is_shutdown():
            try:
                readed = input("")
                self.rate = float(readed)
                inputs.changed = True
            except NameError as e:
                print e
            except SyntaxError as e:
                print e

if __name__ == "__main__":
    rospy.init_node('dynamic_publisher', anonymous=False)

    try:
        rate = rospy.Rate(1.0)
        _pub = rospy.Publisher('/topicA', Int32, queue_size=1)

        inputs = InputThread()
        inputs.start()

        while not rospy.is_shutdown():

            if inputs.changed:
                rate = rospy.Rate(inputs.rate)
                inputs.changed = False

            _pub.publish(1)
            rate.sleep()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
