#!/usr/bin/env python

import rospy


def cb(msg):
    pass

if __name__ == "__main__":
    rospy.init_node('tug_observer', anonymous=False)


    try:
        sub = []
        sub.append(rospy.Subscriber('/topic_0', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_1', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_2', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_3', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_4', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_6', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_7', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_8', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_9', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_10', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_11', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_12', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_13', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_14', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_16', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_17', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_18', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_19', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_20', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_21', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_22', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_23', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_24', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_26', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_27', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_28', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_29', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_30', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_31', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_32', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_33', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_34', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_36', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_37', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_38', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_39', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_40', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_41', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_42', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_43', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_44', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_46', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_47', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_48', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_49', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_50', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_51', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_52', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_53', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_54', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_56', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_57', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_58', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_59', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_60', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_61', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_62', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_63', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_64', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_66', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_67', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_68', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_69', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_70', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_71', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_72', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_73', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_74', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_76', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_77', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_78', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_79', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_80', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_81', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_82', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_83', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_84', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_86', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_87', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_88', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_89', rospy.AnyMsg, cb, queue_size=1))

        sub.append(rospy.Subscriber('/topic_90', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_91', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_92', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_93', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_94', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_96', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_97', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_98', rospy.AnyMsg, cb, queue_size=1))
        sub.append(rospy.Subscriber('/topic_99', rospy.AnyMsg, cb, queue_size=1))

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.logwarn('observer node stopped')

