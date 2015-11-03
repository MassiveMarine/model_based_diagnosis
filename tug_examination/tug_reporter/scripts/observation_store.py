#!/usr/bin/env python
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

    def add_observation(self, type, resource, observations, time_stamp):
        observation_element = tuple([type, resource])
        should_replace = False
        if self._observation_timings.has_key(observation_element):
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

            self._observations_int[observation_element] = tmp_observations_int
            self._observations_string[observation_element] = tmp_observations_string

        self._has_changed = should_replace

    def has_observation(self, type, resource, observation):
        observation_element = tuple([type, resource])
        if isinstance(observation, ObservationWithNumber):
            if self._observations_int.has_key(observation_element):
                return observation.number in self._observations_int[observation_element]
        else:
            if isinstance(observation, ObservationWithString):
                if self._observations_string.has_key(observation_element):
                    return observation.message in self._observations_string[observation_element]
        return False

    def has_changed(self):
        return self._has_changed