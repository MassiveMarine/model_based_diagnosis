#!/usr/bin/env python
from collections import deque

__author__ = 'clemens'


class ObservationWithNumber(object):
    def __init__(self, number):
        self.number = number


class ObservationWithString(object):
    def __init__(self, message):
        self.message = message


class ObservationContainer(object):
    pass


class ObservationStore(object):
    def __init__(self):
        # map between observations and time
        self._observation_timings = {}
        # map for the <type, resource> -> observations
        self._observations_int = {}
        self._observations_string = {}
        self._has_changed = False

    def observations_used(self, observations_of_rule):
        for obs in observations_of_rule:
            observation_element = tuple([obs[0], obs[1]])

            window = 1
            if not obs[2] is None:
                window = obs[2]

            replace_window = False
            if observation_element in self._observations_int:
                if self._observations_int[observation_element].maxlen < window:
                    replace_window = True
            else:
                replace_window = True

            if replace_window:
                self._observations_int[observation_element] = deque(maxlen=window)
                self._observations_string[observation_element] = deque(maxlen=window)

    def add_observation(self, type, resource, observations, time_stamp):
        observation_element = tuple([type, resource])
        should_replace = False

        if observation_element not in self._observations_int:
            return

        if observation_element in self._observation_timings:
            if self._observation_timings[observation_element] < time_stamp:
                should_replace = True
        else:
            should_replace = True

        if should_replace:
            self._observation_timings[observation_element] = time_stamp
            tmp_observations_int = set()
            tmp_observations_string = set()
            for obs in observations:
                tmp_observations_int.add(obs.observation)
                tmp_observations_string.add(obs.observation_msg)

            self._observations_int[observation_element].append(tmp_observations_int)
            self._observations_string[observation_element].append(tmp_observations_string)
        self._has_changed = should_replace

    def _is_observed(self, occurences, window_size, observation_queue, element_to_count):
        if len(observation_queue) == 0:
            return False

        start_index = 0
        if len(observation_queue) > window_size:
            start_index = len(observation_queue) - window_size

        count = 0
        for index, obs_set in enumerate(observation_queue):
            if index >= start_index:
                if element_to_count in obs_set:
                    count += 1

        return count >= occurences

    def has_observation(self, type, resource, observation, occurences, window_size):
        if occurences is None:
            occurences = 1

        if window_size is None:
            window_size = occurences

        observation_element = tuple([type, resource])

        if isinstance(observation, ObservationWithNumber):
            if observation_element in self._observations_int:
                return self._is_observed(occurences, window_size, self._observations_int[observation_element],
                                         observation.number)
        else:
            if isinstance(observation, ObservationWithString):
                if observation_element in self._observations_string:
                    return self._is_observed(occurences, window_size, self._observations_string[observation_element],
                                             observation.message)

        return False

    def has_changed(self):
        return self._has_changed
