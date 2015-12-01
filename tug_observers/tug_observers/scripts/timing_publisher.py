#!/usr/bin/env python

import rospy
from std_msgs.msg import Header

if __name__ == "__main__":
    rospy.init_node('timing_publisher', anonymous=False)

    try:
        pubA = rospy.Publisher('/topicA', Header, queue_size=1)
        rateA = rospy.Rate(0.5)
        pubB = rospy.Publisher('/topicB', Header, queue_size=1)

        while not rospy.is_shutdown():

            pubA.publish(rospy.Header(stamp=rospy.Time.now()))
            rospy.sleep(rospy.Duration(1.0))
            pubA.publish(rospy.Header(stamp=rospy.Time.now()))
            rospy.sleep(rospy.Duration(0.5))
            pubB.publish(rospy.Header(stamp=rospy.Time.now()))
            rateA.sleep()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
