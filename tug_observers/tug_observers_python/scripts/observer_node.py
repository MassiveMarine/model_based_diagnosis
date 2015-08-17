#!/usr/bin/env python

import rospy

import importlib


class PluginManager():

    def __init__(self):
        self._plugins = []

    def load_plugin(self, module_name, class_name, ):
        plugin = None
        try:
            module = importlib.import_module(module_name)

            plugin_ptr = getattr(module, class_name)
            plugin = plugin_ptr()

            self._plugins.append(plugin)

        except Exception as e:
            rospy.logerr(e)

        finally:
            return plugin

    @staticmethod
    def unload_plugin(plugin):
        from tug_observers_python import PluginThread
        if issubclass(plugin.__class__, PluginThread):
            plugin.join(2)
            if plugin.isAlive():
                raise RuntimeError("plugin '" + str(plugin.type) + "' had not "
                                   "stopped in time, it will be killed at the end. "
                                   "Maybe it's because a while-True like loop!")

    def unload_all_plugins(self):
        for plugin in self._plugins:
            try:
                self.unload_plugin(plugin)
            except RuntimeError as e:
                rospy.logerr(e)

    @staticmethod
    def initialize_plugin(plugin, data):
        try:
            plugin.initialize(data)
        except:
            pass

    def get_plugin_list(self):
        return list(self._plugins)


if __name__ == "__main__":
    rospy.init_node('tug_observer', anonymous=False)

    # data = rospy.get_param('/tug_observer_node/setup')
    # print data
    try:
        rospy.loginfo("starting " + rospy.get_name())

        manager = PluginManager()
        new_plugin = manager.load_plugin('hz_plugin', 'Hz')
        manager.initialize_plugin(new_plugin, ['/test', '/test2'])

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        manager.unload_all_plugins()
        rospy.logwarn( 'observer node stopped')

