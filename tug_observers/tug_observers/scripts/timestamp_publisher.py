#!/usr/bin/env python

import rospy
from threading import Thread
from tug_observers_msgs.msg import observer_error, resource_error


class InputThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.setDaemon(True)
        self.delay = 1
        self.changed = False

    def run(self):
        while not rospy.is_shutdown():
            try:
                readed = input("seconds: ")
                self.delay = float(readed)
                inputs.changed = True
            except NameError as e:
                print e
            except SyntaxError as e:
                print e

if __name__ == "__main__":
    rospy.init_node('timestamp_publisher', anonymous=False)

    try:
        rate = rospy.Rate(10.0)
        _pub = rospy.Publisher('/topicA', observer_error, queue_size=1)

        _observer_error = observer_error(type='timestamp_publisher', resource=' ')
        _observer_error.error_msg = resource_error(error_msg='No State fits',
                                                   verbose_error_msg='No state can be found for the measured results',
                                                   error=resource_error.NO_STATE_FITS)
        inputs = InputThread()
        inputs.start()
        delay = rospy.Duration(0)

        while not rospy.is_shutdown():

            if inputs.changed:
                delay = rospy.Duration(inputs.delay)
                inputs.changed = False
            stamp = rospy.Time.now() - delay

            _observer_error.header = rospy.Header(stamp=stamp)

            # temp.publish(_observer_error)

            _pub.publish(_observer_error)
            rate.sleep()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
