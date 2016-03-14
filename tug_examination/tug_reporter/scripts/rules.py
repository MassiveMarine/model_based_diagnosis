#!/usr/bin/env python
import subprocess
import smtplib
from email.mime.text import MIMEText
from tug_python_utils import YamlHelper
from observation_store import ObservationWithNumber
from observation_store import ObservationWithString
from observation_store import ObservationContainer
from diagnosis_store import DiagnosisContainer
import rospy
import dynamic_reconfigure.client

__author__ = 'clemens'


class Rule(object):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot):
        self._positive_observations = positive_observations
        self._negative_observations = negative_observations
        if any(obs in self._positive_observations for obs in self._negative_observations):
            raise ValueError("positive observations and negative observations are not allowed to intersect")

        self._positive_possible_faulty_resources = positive_possible_faulty_resources
        self._negative_possible_faulty_resources = negative_possible_faulty_resources
        if any(obs in self._positive_possible_faulty_resources for obs in self._negative_possible_faulty_resources):
            raise ValueError(
                "positive possible faulty resources and possible faulty resources are not allowed to intersect")
        self._last_called = None
        self._recall_duration = recall_duration
        self._is_single_shot = is_single_shot
        self._was_triggered = False

    def observations_to_use(self):
        result = set()
        for p_obs in self._positive_observations:
            result.add((p_obs.type, p_obs.resource, p_obs.window_size))

        for n_obs in self._negative_observations:
            result.add((n_obs.type, n_obs.resource, n_obs.window_size))

        return result

    def resources_to_use(self):
        result = set()
        for p_obs in self._positive_possible_faulty_resources:
            result.add((p_obs.resource, p_obs.window_size))

        for n_obs in self._negative_possible_faulty_resources:
            result.add((n_obs.resource, n_obs.window_size))

        return result

    def can_trigger(self, observation_store, diagnosis_store):
        if self._is_single_shot and self._was_triggered:
            return False

        if self._last_called is not None and self._recall_duration is not None \
                and self._last_called + self._recall_duration > rospy.Time.now():
            return False

        for p_obs in self._positive_observations:
            if not observation_store.has_observation(p_obs.type, p_obs.resource, p_obs.observation,
                                                     p_obs.occurrences, p_obs.window_size):
                return False

        for n_obs in self._negative_observations:
            if observation_store.has_observation(n_obs.type, n_obs.resource, n_obs.observation,
                                                 n_obs.occurrences, n_obs.window_size):
                return False

        for p_diag in self._positive_possible_faulty_resources:
            if not diagnosis_store.possible_faulty(p_diag.resource, p_diag.occurrences, p_diag.window_size):
                return False

        for n_diag in self._negative_possible_faulty_resources:
            if diagnosis_store.possible_faulty(n_diag.resource, n_diag.occurrences, n_diag.window_size):
                return False

        return True

    def trigger_intern(self):
        self._was_triggered = True
        self._last_called = rospy.Time.now()


class PrintRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, message):
        super(PrintRule, self).__init__(positive_observations, negative_observations,
                                        positive_possible_faulty_resources,
                                        negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._message = message

    def trigger(self):
        super(PrintRule, self).trigger_intern()
        print self._message


class ProcessRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, process):
        super(ProcessRule, self).__init__(positive_observations, negative_observations,
                                          positive_possible_faulty_resources,
                                          negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._process = process

    def trigger(self):
        super(ProcessRule, self).trigger_intern()
        subprocess.call(self._process.split(" "))


class EMailRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, host, port, username,
                 password, subject, to_address, from_address, content):
        super(EMailRule, self).__init__(positive_observations, negative_observations,
                                        positive_possible_faulty_resources,
                                        negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._server = smtplib.SMTP(host, port)
        self._subject = subject
        self._to_address = to_address
        self._from_address = from_address
        self._content = content
        self._host = host
        self._port = port
        self._username = username
        self._password = password

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


class LogFileRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, log_file, log_entry):
        super(LogFileRule, self).__init__(positive_observations, negative_observations,
                                          positive_possible_faulty_resources,
                                          negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._log_file = open(log_file, 'a')
        self._log_entry = log_entry

    def trigger(self):
        super(LogFileRule, self).trigger_intern()
        self._log_file.write(self._log_entry + "\n")
        self._log_file.flush()


class ServiceRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, service_name, service_type,
                 call_msg):
        super(ServiceRule, self).__init__(positive_observations, negative_observations,
                                          positive_possible_faulty_resources,
                                          negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._service_name = service_name
        self._service_type = service_type
        self._call_msg = call_msg

    def trigger(self):
        super(ServiceRule, self).trigger_intern()
        subprocess.call(['rosservice', 'call', self._service_name, '\"' + self._call_msg + '\"'])


class ParameterRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, paramter_name, paramter_value):
        super(ParameterRule, self).__init__(positive_observations, negative_observations,
                                            positive_possible_faulty_resources,
                                            negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._paramter_name = paramter_name
        self._paramter_value = paramter_value

    def trigger(self):
        super(ParameterRule, self).trigger_intern()
        subprocess.call(['rosparam', 'set', self._paramter_name, self._paramter_value])


class DynamicParameterRule(Rule):
    def __init__(self, positive_observations, negative_observations, positive_possible_faulty_resources,
                 negative_possible_faulty_resources, recall_duration, is_single_shot, node_name, paramter_name,
                 paramter_value):
        super(DynamicParameterRule, self).__init__(positive_observations, negative_observations,
                                                   positive_possible_faulty_resources,
                                                   negative_possible_faulty_resources, recall_duration, is_single_shot)
        self._client = dynamic_reconfigure.client.Client(node_name)
        self._paramter_name = paramter_name
        self._paramter_value = paramter_value

    def trigger(self):
        super(DynamicParameterRule, self).trigger_intern()
        self._client.update_configuration({self._paramter_name: self._paramter_value})


class RuleFactory(object):
    _factory_map = {
        'print': lambda config: PrintRuleFactory.instantiate_rule(config),
        'process': lambda config: ProcessRuleFactory.instantiate_rule(config),
        'email': lambda config: EMailRuleFactory.instantiate_rule(config),
        'logfile': lambda config: LogFileRuleFactory.instantiate_rule(config),
        'service': lambda config: ServiceRuleFactory.instantiate_rule(config),
        'paramter': lambda config: ParameterRuleFactory.instantiate_rule(config),
        'dynamic_paramter': lambda config: DynamicParameterRuleFactory.instantiate_rule(config)
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
            if YamlHelper.has_param(obs, 'occurrences'):
                the_obs.occurrences = YamlHelper.get_param(obs, 'occurrences')
            else:
                the_obs.occurrences = None
            if YamlHelper.has_param(obs, 'window_size'):
                the_obs.window_size = YamlHelper.get_param(obs, 'window_size')
            else:
                the_obs.window_size = None

            result.add(the_obs)
        return result

    @staticmethod
    def pars_diag(resources):
        result = set()
        for res in resources:
            the_res = DiagnosisContainer()
            the_res.resource = YamlHelper.get_param(res, 'resource')
            if YamlHelper.has_param(res, 'occurrences'):
                the_res.occurrences = YamlHelper.get_param(res, 'occurrences')
            else:
                the_res.occurrences = None
            if YamlHelper.has_param(res, 'window_size'):
                the_res.window_size = YamlHelper.get_param(res, 'window_size')
            else:
                the_res.window_size = None
            result.add(the_res)
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
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        message = YamlHelper.get_param(config, 'message')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return PrintRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                         negative_possible_faulty_resources, recall_duration, is_single_shot, message)


class ProcessRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        process = YamlHelper.get_param(config, 'process')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return ProcessRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                           negative_possible_faulty_resources, recall_duration, is_single_shot, process)


class EMailRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
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
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return EMailRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                         negative_possible_faulty_resources, recall_duration, is_single_shot, host, port,
                         username, password, subject, to_address, from_address, content)


class LogFileRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        log_file = YamlHelper.get_param(config, 'log_file')
        log_entry = YamlHelper.get_param(config, 'log_entry')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return LogFileRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                           negative_possible_faulty_resources, recall_duration, is_single_shot, log_file, log_entry)


class ServiceRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        service_name = YamlHelper.get_param(config, 'service_name')
        service_type = YamlHelper.get_param(config, 'service_type')
        call_msg = YamlHelper.get_param(config, 'call_msg')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return ServiceRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                           negative_possible_faulty_resources, recall_duration, is_single_shot, service_name,
                           service_type, call_msg)


class ParameterRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        paramter_name = YamlHelper.get_param(config, 'paramter_name')
        paramter_value = YamlHelper.get_param(config, 'paramter_value')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return ParameterRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                             negative_possible_faulty_resources, recall_duration, is_single_shot, paramter_name,
                             paramter_value)


class DynamicParameterRuleFactory(RuleFactory):
    @staticmethod
    def instantiate_rule(config):
        is_single_shot = False
        if YamlHelper.has_param(config, 'single_shot'):
            is_single_shot = YamlHelper.get_param(config, 'single_shot')
        node_name = YamlHelper.get_param(config, 'node_name')
        paramter_name = YamlHelper.get_param(config, 'paramter_name')
        paramter_value = YamlHelper.get_param(config, 'paramter_value')
        positive_observations = RuleFactory.pars_positive_observations(config)
        negative_observations = RuleFactory.pars_negative_observations(config)
        positive_possible_faulty_resources = RuleFactory.pars_positive_possible_faulty_resources(config)
        negative_possible_faulty_resources = RuleFactory.pars_negative_possible_faulty_resources(config)
        recall_duration = None
        if YamlHelper.has_param(config, 'recall_duration'):
            recall_duration = rospy.Duration(YamlHelper.get_param(config, 'recall_duration'))

        return DynamicParameterRule(positive_observations, negative_observations, positive_possible_faulty_resources,
                                    negative_possible_faulty_resources, recall_duration, is_single_shot, node_name,
                                    paramter_name,
                                    paramter_value)
