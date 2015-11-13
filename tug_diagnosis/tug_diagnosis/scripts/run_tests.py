#!/usr/bin/env python
import rostest
PKG = 'tug_diagnosis'
import sys

# print sys.path
sys.path.append('/home/stefan/ros/diag_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis/scripts')
# import roslib; roslib.load_manifest(PKG)

# print sys.path

from pymbd.benchmark.tug_description_parser.observers.hz_observer import TestHzObserver



if __name__ == '__main__':

    rostest.rosrun(PKG, 'test_hz_observer', TestHzObserver)