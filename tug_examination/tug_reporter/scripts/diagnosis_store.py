#!/usr/bin/env python

__author__ = 'clemens'


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

    def mode_string_possible(self, resource, mode):
        if resource in self._possibe_resource_modes_string:
            return mode in self._possibe_resource_modes_string[resource]

        return False

    def mode_int_possible(self, resource, mode):
        if resource in self._possibe_resource_modes_string:
            return mode in self._possibe_resource_modes_string[resource]

        return False

    def possible_faulty(self, resource):
        if resource in self._possibe_resource_modes_int:
            for mode in self._possibe_resource_modes_int[resource]:
                if mode < 0:
                    return True

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
                        if (resource_assignment.resource not in tmp_possibe_resource_modes_string or
                                tmp_possibe_resource_modes_string[resource_assignment.resource] is None):
                            tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                        tmp_possibe_resource_modes_string[resource_assignment.resource].add(
                            resource_assignment.mode_msg)

                        if (resource_assignment.resource not in tmp_possibe_resource_modes_int or
                                tmp_possibe_resource_modes_int[resource_assignment.resource] is None):
                            tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                        tmp_possibe_resource_modes_int[resource_assignment.resource].add(resource_assignment.mode)

            for diagnosis in diagnoses:
                self._diagnoses[type] = diagnosis
                for resource_assignment in diagnosis.diagnosis:
                    if (resource_assignment.resource not in tmp_possibe_resource_modes_string or
                            tmp_possibe_resource_modes_string[resource_assignment.resource] is None):
                        tmp_possibe_resource_modes_string[resource_assignment.resource] = set()
                    tmp_possibe_resource_modes_string[resource_assignment.resource].add(resource_assignment.mode_msg)

                    if (resource_assignment.resource not in tmp_possibe_resource_modes_int or
                            tmp_possibe_resource_modes_int[resource_assignment.resource] is None):
                        tmp_possibe_resource_modes_int[resource_assignment.resource] = set()
                    tmp_possibe_resource_modes_int[resource_assignment.resource].add(resource_assignment.mode)

            for resources in tmp_possibe_resource_modes_string.keys():
                self._possibe_resource_modes_string[resources] = tmp_possibe_resource_modes_string[resources]

            for resources in tmp_possibe_resource_modes_int.keys():
                self._possibe_resource_modes_int[resources] = tmp_possibe_resource_modes_int[resources]

        self._has_changed = should_replace

    def has_changed(self):
        return self._has_changed
