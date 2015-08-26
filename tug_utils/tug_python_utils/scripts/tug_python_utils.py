#!/usr/bin/env python


class YamlHelper():
    @staticmethod
    def get_param(config, name):
        if config.has_key(name):
            return config[name]
        print "ERROR"
        raise KeyError("'" + str(name) + "' not found in config!")


    @staticmethod
    def has_key(config, name):
        return config.has_key(name)
