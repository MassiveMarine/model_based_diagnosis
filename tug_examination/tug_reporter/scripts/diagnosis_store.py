#!/usr/bin/env python
from collections import deque

__author__ = 'clemens'


class DiagnosisContainer(object):
    pass


class DiagnosisStore(object):
    def __init__(self):
        # map between timing and diagnosis
        self._diagnosis_timing = {}
        # map between resources and mode assignements which are possible
        self._possibe_resource_modes_string = {}
        self._possibe_resource_modes_int = {}
        # map of type to current diagnosis
        self._diagnoses = {}
        self._has_changed = False

    def _is_possible_faulty(self, occurences, window_size, resource_modes_queue):
        if len(resource_modes_queue) == 0:
            return False

        start_index = 0
        if len(resource_modes_queue) > window_size:
            start_index = len(resource_modes_queue) - window_size

        count = 0
        for index, resource_assignments in enumerate(resource_modes_queue):
            if index >= start_index:
                for mode in resource_assignments:
                    if mode < 0:
                        count += 1
                        break

        return count >= occurences

    def mode_string_possible(self, resource, mode):
        if resource in self._possibe_resource_modes_string:
            return mode in self._possibe_resource_modes_string[resource]

        return False

    def mode_int_possible(self, resource, mode):
        if resource in self._possibe_resource_modes_string:
            return mode in self._possibe_resource_modes_string[resource]

        return False

    def possible_faulty(self, resource, occurences, window_size):
        if occurences is None:
            occurences = 1

        if window_size is None:
            window_size = occurences

        if resource in self._possibe_resource_modes_int:
            return self._is_possible_faulty(occurences, window_size, self._possibe_resource_modes_int[resource])

        return False

    def add_diagnosis(self, type, diagnoses, time_stamp):
        should_replace = False
        if type in self._diagnosis_timing:
            if self._diagnosis_timing[type] < time_stamp:
                should_replace = True
        else:
            should_replace = True

        if should_replace:
            self._diagnosis_timing[type] = time_stamp
            tmp_possibe_resource_modes_string = {}
            tmp_possibe_resource_modes_int = {}
            for other_type, dig in self._diagnoses.iteritems():
                if other_type != type:
                    for resource_assignment in dig.diagnosis:
                        if resource_assignment.resource not in tmp_possibe_resource_modes_string:
                            tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                        if tmp_possibe_resource_modes_string[resource_assignment.resource] is None:
                            tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                        tmp_possibe_resource_modes_string[resource_assignment.resource].add(
                            resource_assignment.mode_msg)

                        if resource_assignment.resource not in tmp_possibe_resource_modes_int:
                            tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                        if tmp_possibe_resource_modes_int[resource_assignment.resource] is None:
                            tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                        tmp_possibe_resource_modes_int[resource_assignment.resource].add(resource_assignment.mode)

            for diagnosis in diagnoses:
                self._diagnoses[type] = diagnosis
                for resource_assignment in diagnosis.diagnosis:
                    if resource_assignment.resource not in tmp_possibe_resource_modes_string:
                        tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                    if tmp_possibe_resource_modes_string[resource_assignment.resource] is None:
                        tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                    tmp_possibe_resource_modes_string[resource_assignment.resource].add(resource_assignment.mode_msg)

                    if resource_assignment.resource not in tmp_possibe_resource_modes_int:
                        tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                    if tmp_possibe_resource_modes_int[resource_assignment.resource] is None:
                        tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                    tmp_possibe_resource_modes_int[resource_assignment.resource].add(resource_assignment.mode)

            for resources in tmp_possibe_resource_modes_string.keys():
                if resources not in self._possibe_resource_modes_string:
                    continue
                self._possibe_resource_modes_string[resources].append(tmp_possibe_resource_modes_string[resources])

            for resources in tmp_possibe_resource_modes_int.keys():
                if resources not in self._possibe_resource_modes_int:
                    continue
                self._possibe_resource_modes_int[resources].append(tmp_possibe_resource_modes_int[resources])

        self._has_changed = should_replace

    def has_changed(self):
        return self._has_changed

    def resources_used(self, resources_of_rule):
        for res in resources_of_rule:
            the_resource = res[0]
            window = 1
            if not res[1] is None:
                window = res[1]

            replace_window = False
            if res in self._possibe_resource_modes_int:
                if self._possibe_resource_modes_int[the_resource].maxlen < window:
                    replace_window = True
            else:
                replace_window = True

            if replace_window:
                self._possibe_resource_modes_int[the_resource] = deque(maxlen=window)
                self._possibe_resource_modes_string[the_resource] = deque(maxlen=window)
