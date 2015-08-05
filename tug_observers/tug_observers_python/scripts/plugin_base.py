#!/usr/bin/env python
import rospy


class PluginBase:
    def __init__(self):
        pass

    def run(self):
        rospy.logerr('PluginBase.run')
