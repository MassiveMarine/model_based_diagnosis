#!/usr/bin/env python

import rospy
from threading import Thread
from tug_observers_msgs.msg import observation, observation_info


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
        _pub = rospy.Publisher('/topicA', observation_info, queue_size=1)

        _observation_info = observation_info(type='timestamp_publisher', resource=' ')
        _observation_info.observation = [observation(observation_msg='No State fits',
                                                    verbose_observation_msg='No state can be found for the measured results',
                                                    observation=observation.NO_STATE_FITS)]
        inputs = InputThread()
        inputs.start()
        delay = rospy.Duration(0)

        while not rospy.is_shutdown():

            if inputs.changed:
                delay = rospy.Duration(inputs.delay)
                inputs.changed = False
            stamp = rospy.Time.now() - delay

            _observation_info.header = rospy.Header(stamp=stamp)

            # temp.publish(_observer_error)

            _pub.publish(_observation_info)
            rate.sleep()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
