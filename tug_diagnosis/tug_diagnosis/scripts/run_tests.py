#!/usr/bin/env python
import rostest
PKG = 'tug_diagnosis'

import sys
import os

# add own folder to sys path. No pymbd is accessible!
sys.path.append(os.path.dirname(__file__))

from pymbd.benchmark.tug_description_parser.observers.base_observer import TestCalleridsObserver
from pymbd.benchmark.tug_description_parser.observers.hz_observer import TestHzObserver
from pymbd.benchmark.tug_description_parser.observers.timestamp_observer import TestTimestampObserver
from pymbd.benchmark.tug_description_parser.observers.timeout_observer import TestTimeoutObserver
from pymbd.benchmark.tug_description_parser.observers.resources_observer import TestResourcesObserver
from pymbd.benchmark.tug_description_parser.observers.timing_observer import TestTimingObserver
from pymbd.benchmark.tug_description_parser.observers.movement_observer import TestMovementObserver
from pymbd.benchmark.tug_description_parser.observers.scores_observer import TestScoresObserver
from pymbd.benchmark.tug_description_parser.observers.activated_observer import TestActivatedObserver


if __name__ == '__main__':

    print "Hallo"

    rostest.rosrun(PKG, 'test_base_observer', TestCalleridsObserver)
    rostest.rosrun(PKG, 'test_hz_observer', TestHzObserver)
    rostest.rosrun(PKG, 'test_timestamp_observer', TestTimestampObserver)
    rostest.rosrun(PKG, 'test_timeout_observer', TestTimeoutObserver)
    rostest.rosrun(PKG, 'test_resources_observer', TestResourcesObserver)
    rostest.rosrun(PKG, 'test_timing_observer', TestTimingObserver)
    rostest.rosrun(PKG, 'test_movement_observer', TestMovementObserver)
    rostest.rosrun(PKG, 'test_scores_observer', TestScoresObserver)
    rostest.rosrun(PKG, 'test_activated_observer', TestActivatedObserver)
