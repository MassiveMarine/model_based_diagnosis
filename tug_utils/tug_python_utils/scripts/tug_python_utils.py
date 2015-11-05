#!/usr/bin/env python


class YamlHelper():
    @staticmethod
    def get_param(config, name):
        if name in config:
            return config[name]
        print "ERROR"
        raise KeyError("'" + str(name) + "' not found in config!")
        
    @staticmethod
    def get_param_with_default(config, name, default):
        if name in config:
            return config[name]
        return default
        
    @staticmethod
    def has_key(config, name):
        return name in config
        
    @staticmethod
    def has_param(config, name):
        return name in config
