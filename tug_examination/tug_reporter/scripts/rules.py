#!/usr/bin/env python
import subprocess
import smtplib
from email.mime.text import MIMEText
from tug_python_utils import YamlHelper
from observation_store import ObservationWithNumber
from observation_store import ObservationWithString
from observation_store import ObservationContainer
import rospy

__author__ = 'clemens'


class Rule(object):

    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration):
        self._positive_observations = positive_observations
        self._negative_observations = negative_observations
        self._positive_possible_faulty_resources = positive_possible_faulty_resources
        self._negative_possible_faulty_resources = negative_possible_faulty_resources
        self._last_called = None
        self._recall_duration = recall_duration

    def can_trigger(self, observation_store, diagnosis_store):
        if self._last_called is not None and self._last_called + self._recall_duration > rospy.Time.now():
            return False

        for p_obs in self._positive_observations:
            if not observation_store.has_observation(p_obs.type, p_obs.resource, p_obs.observation):
                return False

        for n_obs in self._negative_observations:
            if observation_store.has_observation(p_obs.type, p_obs.resource, p_obs.observation):
                return False

        for p_diag in self._positive_possible_faulty_resources:
            if not diagnosis_store.possible_faulty(p_diag):
                return False

        for n_diag in self._negative_possible_faulty_resources:
            if diagnosis_store.possible_faulty(n_diag):
                return False

        return True

    def trigger_intern(self):
        self._last_called = rospy.Time.now()


class PrintRule(Rule):

    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, message):
        super(PrintRule, self).__init__(positive_observations, negative_observations,
                                        positive_possible_faulty_resources,
                                        negative_possible_faulty_resources, recall_duration)
        self._message = message

    def trigger(self):
        super(PrintRule, self).trigger_intern()
        print self._message


class ProcessRule(Rule):

    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, process):
        super(ProcessRule, self).__init__(positive_observations, negative_observations,
                                          positive_possible_faulty_resources,
                                          negative_possible_faulty_resources, recall_duration)
        self._process = process

    def trigger(self):
        super(ProcessRule, self).trigger_intern()
        subprocess.call(self._process.split(" "))


class EMailRule(Rule):

    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, host, port, username, password, subject,
                 to_address, from_address, content):
        super(EMailRule, self).__init__(positive_observations, negative_observations,
                                        positive_possible_faulty_resources,
                                        negative_possible_faulty_resources, recall_duration)
        self._server = smtplib.SMTP(host, port)
        self._subject = subject
        self._to_address = to_address
        self._from_address = from_address
        self._content = content

    def trigger(self):
        super(EMailRule, self).trigger_intern()
        server = smtplib.SMTP(self._host, self._port)
        server.login(self._username, self._password)

        msg = MIMEText(self._content)
        msg['Subject'] = self._subject
        msg['From'] = self._from_address
        msg['To'] = self._to_address

        server.sendmail(self._from_address, [self._to_address], msg.as_string())

        server.quit()


class RuleFactory(object):

    _factory_map = {
                    'print': lambda config: PrintRuleFactory.instantiate_rule(config),
                    'process': lambda config: ProcessRuleFactory.instantiate_rule(config),
                    'email': lambda config: EMailRuleFactory.instantiate_rule(config)
                }

    @staticmethod
    def create_rule(rule_type, config):
        if rule_type not in RuleFactory._factory_map:
            raise KeyError("'" + str(rule_type) + "' not known!")

        return RuleFactory._factory_map[rule_type](config)

    @staticmethod
    def pars_observation(observations):
        result = set()
        for obs in observations:
            the_obs = ObservationContainer()
            the_obs.type = YamlHelper.get_param(obs, 'type')
            the_obs.resource = YamlHelper.get_param(obs, 'resource')
            if YamlHelper.has_param(obs, 'observation'):
                the_obs.observation = ObservationWithNumber(YamlHelper.get_param(obs, 'observation'))
            else:
                the_obs.observation = ObservationWithString(YamlHelper.get_param(obs, 'observation_msg'))

            result.add(the_obs)
        return result

    @staticmethod
    def pars_diag(observations):
        result = set()
        for obs in observations:
            result.add(YamlHelper.get_param(obs, 'resource'))
        return result

    @staticmethod
    def pars_positive_observations(config):
        return RuleFactory.pars_observation(YamlHelper.get_param_with_default(config, 'positive_observations', []))

    @staticmethod
    def pars_negative_observations(config):
        return RuleFactory.pars_observation(YamlHelper.get_param_with_default(config, 'negative_observations', []))

    @staticmethod
    def pars_positive_possible_faulty_resources(config):
        return RuleFactory.pars_diag(YamlHelper.get_param_with_default(config,
                                                                       'positive_possible_faulty_resources', []))

    @staticmethod
    def pars_negative_possible_faulty_resources(config):
        return RuleFactory.pars_diag(YamlHelper.get_param_with_default(config,
                                                                       'negative_possible_faulty_resources', []))


class PrintRuleFactory(RuleFactory):

    @staticmethod
    def instantiate_rule(config):
        message = YamlHelper.get_param(config, 'message')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return PrintRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                         negative_possible_faulty_resources, recall_duration, message)


class ProcessRuleFactory(RuleFactory):

    @staticmethod
    def instantiate_rule(config):
        process = YamlHelper.get_param(config, 'process')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return ProcessRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                           negative_possible_faulty_resources, recall_duration, process)


class EMailRuleFactory(RuleFactory):

    @staticmethod
    def instantiate_rule(config):
        host = YamlHelper.get_param(config, 'host')
        port = YamlHelper.get_param(config, 'port')
        username = YamlHelper.get_param(config, 'username')
        password = YamlHelper.get_param(config, 'password')
        subject = YamlHelper.get_param(config, 'subject')
        to_address = YamlHelper.get_param(config, 'to_address')
        from_address = YamlHelper.get_param(config, 'from_address')
        content = YamlHelper.get_param(config, 'content')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return EMailRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                         negative_possible_faulty_resources, recall_duration, host, port, username, password, subject,
                         to_address, from_address, content)
