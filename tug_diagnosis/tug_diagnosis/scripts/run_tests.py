#!/usr/bin/env python
import rostest

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
from pymbd.benchmark.tug_description_parser.observers.velocity_observer import TestVelocityObserver
from pymbd.benchmark.tug_description_parser.observers.scores_observer import TestScoresObserver
from pymbd.benchmark.tug_description_parser.observers.activated_observer import TestActivatedObserver

from pymbd.benchmark.tug_description_parser.model import TestConfigurationValidation
from pymbd.benchmark.tug_description_parser.model import TestModelGenerator

from config_validator import TestDiagnosisConfigValidator

PKG = 'tug_diagnosis'

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_base_observer', TestCalleridsObserver)
    rostest.rosrun(PKG, 'test_hz_observer', TestHzObserver)
    rostest.rosrun(PKG, 'test_timestamp_observer', TestTimestampObserver)
    rostest.rosrun(PKG, 'test_timeout_observer', TestTimeoutObserver)
    rostest.rosrun(PKG, 'test_resources_observer', TestResourcesObserver)
    rostest.rosrun(PKG, 'test_timing_observer', TestTimingObserver)
    rostest.rosrun(PKG, 'test_velocity_observer', TestVelocityObserver)
    rostest.rosrun(PKG, 'test_scores_observer', TestScoresObserver)
    rostest.rosrun(PKG, 'test_activated_observer', TestActivatedObserver)

    rostest.rosrun(PKG, 'test_configuration_validation', TestConfigurationValidation)
    rostest.rosrun(PKG, 'test_model_generator', TestModelGenerator)

    rostest.rosrun(PKG, 'test_diagnosis_config_validator', TestDiagnosisConfigValidator)
