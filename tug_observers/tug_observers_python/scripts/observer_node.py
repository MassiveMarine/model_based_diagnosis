#!/usr/bin/env python

import rospy
import rospkg
import sys
import importlib


class PluginManager():
    """
    Manager for plugins, which loads, stores the instances and unload them.
    """

    def __init__(self):
        """
        Constructor for plugin manager.
        """
        self._plugins = []

    @staticmethod
    def add_plugin_paths_to_sys_path():
        """
        Add all packages which depends on the plugin base to sys path to be importable.
        """
        rospack = rospkg.RosPack()
        pkgs = rospack.get_depends_on('tug_observers_python')
        for pkg in pkgs:
            new_pkg_path = rospack.get_path(pkg) + '/scripts'
            sys.path = [new_pkg_path] + sys.path

    def load_plugin(self, module_name, class_name):
        """
        Find the plugin, import it, create a instance and add it to the plugins
        list of the manager.
        :param module_name: python script name
        :type module_name: str
        :param class_name: class name
        :type class_name: str
        :return: instance of plugin or None if not found or possible
        """
        plugin = None
        try:
            # self._add_pkg_path_to_sys_path(pkg_name)
            module = importlib.import_module(module_name)

            plugin_ptr = getattr(module, class_name)
            plugin = plugin_ptr()
            self._plugins.append(plugin)
        except Exception as e:
            rospy.logerr(e)
        finally:
            return plugin

    @staticmethod
    def unload_plugin(plugin, timeout=2):
        """
        Unload a plugin.
        :param plugin: instance of the plugin
        :type plugin: PluginBase
        :param timeout: seconds to wait for multi-threaded plugins to stop
        :type timeout: float
        :raise RuntimeError: if plugin do not respond on ros shutdown
        """
        from tug_observers_python import PluginThread
        if issubclass(plugin.__class__, PluginThread):
            plugin.join(timeout)
            if plugin.isAlive():
                raise RuntimeError("plugin '" + str(plugin.type) + "' had not "
                                   "stopped in time, it will be killed at the end. "
                                   "Maybe it's because a while-True like loop!")

    def unload_all_plugins(self):
        """
        Unload all plugins in list.
        """
        for plugin in self._plugins:
            try:
                self.unload_plugin(plugin)
            except RuntimeError as e:
                rospy.logerr(e)

    @staticmethod
    def initialize_plugin(plugin, config):
        try:
            plugin.initialize(config)
        except:
            pass

    def get_plugin_list(self):
        """
        Get all loaded plugins.
        :return: list of all loaded plugins
        """
        return list(self._plugins)


if __name__ == "__main__":
    rospy.init_node('tug_observer', anonymous=False)

    configs = rospy.get_param('/tug_observer_node/setup')

    try:
        rospy.loginfo("starting " + rospy.get_name())

        manager = PluginManager()
        manager.add_plugin_paths_to_sys_path()

        for config in configs:
            plugin = manager.load_plugin(config['type'], config['type'])
            manager.initialize_plugin(plugin, config)

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        manager.unload_all_plugins()
        rospy.logwarn('observer node stopped')

