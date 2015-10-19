#!/usr/bin/env python

# basics
import rospy
from tug_resource_monitor.srv import *
import rosnode
import os
import psutil
from tug_resource_monitor.msg import NodeInfo, NodeInfoArray
from std_msgs.msg import Header
from tug_python_utils import YamlHelper as Config

class NodeData:

    ONLINE = 1
    OFFLINE = 0
    NEED_INIT = 2

    def __init__(self, name, pid=0, process=None, hostname='', state=NEED_INIT):
        self.name = name
        self.pid = pid
        self.process = process
        self.hostname = hostname
        self.cpu_usage = 0.0
        self.mem_usage = 0
        self.error = NodeInfo.NO_ERROR
        self.state = state

    def __str__(self):
        return "PID: " + str(self.pid) + " machine: '" + str(self.hostname) + "' state: " + str(self.state)

    def __repr__(self):
        return str(self)


class TUGResourceMonitor:
    def __init__(self):
        self.own_hostname, node_api = get_hostname()
        self.nodes = dict()
        rospy.logdebug("resource monitor started")
        if rospy.has_param(rospy.get_name() + '/nodes_to_observer'):
            rospy.logdebug("got nodes parameter")
            node_names = rospy.get_param(rospy.get_name() + '/nodes_to_observer')
            for name in node_names:
                self.nodes[name] = NodeData(name=name)

        self.node_infos_pub = rospy.Publisher('diag/node_infos', NodeInfoArray, queue_size=10)

    def run(self, frequency=1.0):
        rate = rospy.Rate(frequency)

        self.waiting_for_nodes_list()
        self.waiting_for_all_nodes_coming_online()

        rospy.loginfo("entering loop with '" + str(frequency) + " Hz'")
        while not rospy.is_shutdown():

            nodes_info_array = NodeInfoArray()
            for node in self.nodes.itervalues():
                self.get_pid_of_node(node)
                self.get_cpu_and_mem_usage_of_node(node)
                nodes_info_array.data.append(NodeInfo(name=node.name, pid=node.pid, hostname=node.hostname,
                                                      cpu=node.cpu_usage, memory=node.mem_usage,
                                                      error=node.error))

            nodes_info_array.header = Header(stamp=rospy.Time.now())
            self.node_infos_pub.publish(nodes_info_array)
            rate.sleep()

        rospy.logerr('done')

    def waiting_for_nodes_list(self):
        rospy.loginfo("waiting for nodes list")
        rate = rospy.Rate(10.0)
        while not self.nodes and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("done")

    def waiting_for_all_nodes_coming_online(self):
        rospy.loginfo("waiting for all nodes coming online")
        rate = rospy.Rate(10.0)
        while next((x for x in self.nodes.values() if x.state == 2), False) and not rospy.is_shutdown():
            node_names_currently_running = rosnode.get_node_names()
            # print node_names_currently_running

            if not set(self.nodes.keys()) - set(node_names_currently_running):
                for node_name in list(set(self.nodes.keys()) & set(node_names_currently_running)):
                    self.nodes[node_name].state = NodeData.ONLINE
            rate.sleep()
        rospy.loginfo("done")

    def get_pid_of_node(self, node, master=rosnode.rosgraph.Master(rosnode.ID)):
        try:
            # check if pid is already known
            if node.pid:
                return

            hostname, node_api = get_hostname_of_nodes(node_name=node.name, master=master)

            # only take nodes, which are running on this machine
            if hostname != self.own_hostname:
                return

            # read out pid of node
            code, msg, pid = rosnode.ServerProxy(node_api).getPid(rosnode.ID)
            if code != 1:
                raise rosnode.ROSNodeException("remote call failed: '" + node.name + "'")

            node.hostname = hostname
            node.pid = pid
            node.error = NodeInfo.NO_ERROR

        except rosnode.ROSNodeException as error:
            rospy.logwarn(str(error))
        except rosnode.ROSNodeIOException as error:
            rospy.logerr(str(error))
        except IOError as error:
            if node.state == NodeData.OFFLINE:
                rospy.logwarn("Connection refused '" + str(node.name) + "'; maybe its not running any more")
            else:
                rospy.logerr(str(error))

    @staticmethod
    def get_cpu_and_mem_usage_of_node(node):
        try:
            if not node.process:
                if not node.pid:
                    return
                node.process = psutil.Process(node.pid)

            node.mem_usage = node.process.get_memory_info()[0]
            node.cpu_usage = node.process.get_cpu_percent(interval=0)
        except psutil.NoSuchProcess:
            rospy.logwarn("pid of node '" + node.name + "' not found")
            node.pid = 0
            node.process = None
            node.cpu_usage = 0.0
            node.mem_usage = 0
            node.error = NodeInfo.ERROR_PID_NOT_FOUND
            node.state = NodeData.OFFLINE


def handle_new_nodes_list(req):
    rospy.logwarn("new nodes list" + str(req.node_names))
    monitor.nodes.clear()
    for name in req.node_names:
        monitor.nodes[name] = NodeData(name=name)
    return NodesInfoResponse()


def get_hostname(master=rosnode.rosgraph.Master(rosnode.ID)):
    return get_hostname_of_nodes(rospy.get_name(), master)


def get_hostname_of_nodes(node_name, master=rosnode.rosgraph.Master(rosnode.ID)):
    # check if master is available
    node_api = rosnode.get_api_uri(master, node_name, skip_cache=True)
    # check if node can be found
    if not node_api:
        raise rosnode.ROSNodeException("cannot find node '" + str(node_name) + "'; maybe its not running any more")

    return rosnode.urlparse.urlparse(node_api).hostname, node_api


if __name__ == "__main__":
    rospy.init_node('tug_resource_monitor_' + str(os.getpid()), anonymous=False)

    try:
        rospy.loginfo("starting " + rospy.get_name())
        monitor = TUGResourceMonitor()
        s = rospy.Service('new_nodes_list', NodesInfo, handle_new_nodes_list)
        monitor.run()

    except KeyboardInterrupt:
        pass
    except rosnode.ROSNodeIOException as e:
        print e
    except rospy.ROSInterruptException:
        pass


########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################

# class NodeMetaData:
#     def __init__(self, name='', pid=0, psutil_Process=None, hostname=''):
#
#         self.name = name
#         self.pid = pid
#         self.psutil_process = psutil_Process
#         self.hostname = hostname
#         self.cpu_usage = 0.0
#         self.mem_usage = 0
#         self.error = NodeInfo.NO_ERROR
#
#     def __str__(self):
#         return "\nNode '" + str(self.name) + "' has PID '" + str(self.pid) + "' and runs on machine '" + str(self.hostname) + "'"
#
#     def __repr__(self):
#         return str(self)
#
#
# class TUGResourceMonitor:
#     def __init__(self):
#         self.master = rosnode.rosgraph.Master(rosnode.ID)
#         node_api = rosnode.get_api_uri(self.master, rospy.get_name())
#         self.own_hostname = rosnode.urlparse.urlparse(node_api).hostname
#
#         self.node_names = []
#         self.node_names_online = []
#         self.node_names_offline = []
#
#         self.node_infos_pub = rospy.Publisher('diag/node_infos', NodeInfoArray, queue_size=10)
#
#     def get_pids_of_nodes(self, nodes=[]):
#         if not nodes:
#             nodes = self.node_names_online
#
#         master = rosnode.rosgraph.Master(rosnode.ID)
#         for node in nodes:
#             try:
#                 node_api = rosnode.get_api_uri(master, node.name)
#
#                 # check if node can be found
#                 if not node_api:
#                     rospy.logwarn("cannot find '" + node.name + "': unknown node")
#                     continue
#
#                 hostname = rosnode.urlparse.urlparse(node_api).hostname
#                 print "2"
#                 # only take nodes, which are running on this machine
#                 if hostname != self.own_hostname:
#                     continue
#
#                 # read out pid of node
#                 code, msg, pid = rosnode.ServerProxy(node_api).getPid(rosnode.ID)
#                 if code != 1:
#                     rospy.logwarn("remote call failed: '" + node + "'")
#
#                 node.hostname = hostname
#                 node.pid = pid
#
#             except rosnode.ROSNodeIOException as error:
#                 rospy.logerr(error)
#             except IOError as error:
#                 rospy.logerr(error)
#
#         return nodes
#
#     def get_cpu_and_mem_usage(self):
#         for node in self.node_names_online:
#             try:
#                 if not node.psutil_process:
#                     if node.pid:
#                         continue
#                     node.psutil_process = psutil.Process(node.pid)
#
#                 node.mem_usage = node.psutil_process.get_memory_info()[0]
#                 node.cpu_usage = node.psutil_process.get_cpu_percent(interval=0)
#
#             except psutil.NoSuchProcess:
#                 node.pid = 0
#                 node.psutil_process = None
#                 node.hostname = ''
#                 node.mem_usage = 0
#                 node.cpu_usage = 0.0
#                 node.psutil_process = None
#                 node.error = NodeInfo.ERROR_PID_NOT_FOUND
#                 self.node_names_offline.append(node)
#                 self.node_names_online = list(set(self.node_names_online) - set(self.node_names_offline))
#
#     def run(self, frequency=1.0):
#
#         rate = rospy.Rate(10.0)
#         rospy.loginfo("waiting for nodes list")
#         while not self.node_names and not rospy.is_shutdown():
#             rate.sleep()
#
#         while not rospy.is_shutdown():
#
#             rospy.loginfo("waiting for all nodes coming online")
#             while not self.node_names_online and not rospy.is_shutdown():
#                 node_names_currently_running = rosnode.get_node_names()
#                 print node_names_currently_running
#                 if not list(set(self.node_names) - set(node_names_currently_running)):
#                     for node_name in self.node_names:
#                         self.node_names_online.append(NodeMetaData(name=node_name))
#                     self.node_names = []
#                 rate.sleep()
#
#             rospy.loginfo("read pids of nodes")
#
#             self.get_pids_of_nodes()
#
#             rospy.loginfo("entering loop with '" + str(frequency) + " Hz'")
#             rate = rospy.Rate(frequency)
#             while not rospy.is_shutdown():
#                 self.get_cpu_and_mem_usage()
#                 nodes_info_array = NodeInfoArray()
#                 nodes_info_array.header = Header(stamp=rospy.Time.now())
#                 for node in self.node_names_online or self.node_names_offline:
#                     nodes_info_array.data.append(NodeInfo(name=node.name, pid=node.pid, hostname=node.hostname,
#                                                               cpu=node.cpu_usage, memory=node.mem_usage,
#                                                               error=node.error))
#                 self.node_infos_pub.publish(nodes_info_array)
#
#                 if self.node_names:
#                     self.node_names_online = []
#                     self.node_names_offline = []
#                     break
#                 rate.sleep()
#
#
# def handle_new_nodes_list(req):
#     rospy.logwarn("new nodes list" + str(req.node_names))
#     monitor.node_names = req.node_names
#     return NodesInfoResponse()
#
# if __name__ == "__main__":
#     rospy.init_node('tug_resource_monitor_' + str(os.getpid()), anonymous=False)
#
#     try:
#         rospy.loginfo("starting " + rospy.get_name())
#
#         monitor = TUGResourceMonitor()
#
#         s = rospy.Service('new_nodes_list', NodesInfo, handle_new_nodes_list)
#
#         monitor.run()
#
#
#     except KeyboardInterrupt:
#         pass
#     except rosnode.ROSNodeIOException as e:
#         print e
#     except rospy.ROSInterruptException:
#         pass


########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
# class TUGResourceMonitor:
#     def __init__(self):
#         self.processes = dict()  # processes are NEVER removed from this dict
#         self.master = rosnode.rosgraph.Master(rosnode.ID)
#         node_api = rosnode.get_api_uri(self.master, rospy.get_name())
#         self.own_hostname = rosnode.urlparse.urlparse(node_api).hostname
#
#         self.node_infos_pub = rospy.Publisher('diag/node_infos', NodeInfoArray, queue_size=10)
#
#     def get_cpu_and_mem_usage(self, pids):
#         process_info = dict()
#
#         for pid in pids:
#             try:
#                 if not self.processes.has_key(pid):
#                     self.processes[pid] = psutil.Process(pid)
#
#                 memory_info = self.processes[pid].get_memory_info()[0]
#                 cpu_percent = self.processes[pid].get_cpu_percent(interval=0)
#
#                 process_info[pid] = [cpu_percent, memory_info]
#             except psutil.NoSuchProcess:
#                 pass
#
#         return process_info
#
#     def get_node_names_and_pids(self):
#         process_info = dict()
#
#         node_names = rosnode.get_node_names()
#         master = rosnode.rosgraph.Master(rosnode.ID)
#
#         for node in node_names:
#             try:
#                 node_api = rosnode.get_api_uri(master, node)
#
#                 # check if node can be found
#                 if not node_api:
#                     rospy.logwarn("cannot find '" + node + "': unknown node")
#                     continue
#
#                 hostname = rosnode.urlparse.urlparse(node_api).hostname
#
#                 # only take nodes, which are running on this machine
#                 if hostname != self.own_hostname:
#                     continue
#
#                 # read out pid of node
#                 code, msg, pid = rosnode.ServerProxy(node_api).getPid(rosnode.ID)
#                 if code != 1:
#                     rospy.logwarn("remote call failed: '" + node + "'")
#
#                 process_info[pid] = [node, hostname]
#
#             except rosnode.ROSNodeIOException as error:
#                 rospy.logerr(error)
#             except IOError as error:
#                 rospy.logerr(error)
#
#         return process_info
#
#     def run(self, frequency=1.0):
#
#         rate = rospy.Rate(frequency)
#         nodes_info = self.get_node_names_and_pids()
#         while not rospy.is_shutdown():
#
#             nodes_info_array = NodeInfoArray()
#             nodes_info_array.header = Header(stamp=rospy.Time.now())
#
#             process_info = self.get_cpu_and_mem_usage(nodes_info.keys())
#
#             for pid, [node, hostname] in nodes_info.iteritems():
#                 try:
#                     nodes_info_array.data.append(NodeInfo(name=node, pid=pid, hostname=hostname,
#                                                           cpu=process_info[pid][0], memory=process_info[pid][1],
#                                                           error=NodeInfo.NO_ERROR))
#                 except KeyError:
#                     rospy.logerr("error with PID of node '" + node + "'")
#                     nodes_info_array.data.append(NodeInfo(name=node, pid=0, hostname='0.0.0.0',
#                                                           cpu=0.0, memory=0,
#                                                           error=NodeInfo.ERROR_PID_NOT_FOUND))
#
#             self.node_infos_pub.publish(nodes_info_array)
#             rate.sleep()
#
# if __name__ == "__main__":
#     rospy.init_node('tug_resource_monitor_' + str(os.getpid()), anonymous=False)
#
#     try:
#         rospy.loginfo("starting " + rospy.get_name())
#         TUGResourceMonitor().run(1)
#     except KeyboardInterrupt:
#         pass
#     except rosnode.ROSNodeIOException as e:
#         print e
#     except rospy.ROSInterruptException:
#         pass
