#!/usr/bin/env python

# basics
import rospy

import rosnode
from tug_resource_monitor.msg import NodeInfo, NodeInfoArray
import psutil
import os
from std_msgs.msg import Header


class TUGResourceMonitor:
    def __init__(self):
        self.processes = dict()  # processes are NEVER removed from this dict
        self.master = rosnode.rosgraph.Master(rosnode.ID)
        node_api = rosnode.get_api_uri(self.master, rospy.get_name())
        self.own_hostname = rosnode.urlparse.urlparse(node_api).hostname

        self.node_infos_pub = rospy.Publisher('diag/node_infos', NodeInfoArray, queue_size=10)

    def get_cpu_and_mem_usage(self, pids):
        process_info = dict()

        for pid in pids:
            try:
                if not self.processes.has_key(pid):
                    self.processes[pid] = psutil.Process(pid)

                memory_info = self.processes[pid].memory_info()[0]
                cpu_percent = self.processes[pid].cpu_percent(interval=0)

                process_info[pid] = [cpu_percent, memory_info]
            except psutil.NoSuchProcess:
                pass

        return process_info

    def get_node_names_and_pids(self):
        process_info = dict()

        node_names = rosnode.get_node_names()
        master = rosnode.rosgraph.Master(rosnode.ID)

        for node in node_names:
            try:
                node_api = rosnode.get_api_uri(master, node)

                # check if node can be found
                if not node_api:
                    rospy.logwarn("cannot find '" + node + "': unknown node")
                    continue

                hostname = rosnode.urlparse.urlparse(node_api).hostname

                # only take nodes, which are running on this machine
                if hostname != self.own_hostname:
                    continue

                # read out pid of node
                code, msg, pid = rosnode.ServerProxy(node_api).getPid(rosnode.ID)
                if code != 1:
                    rospy.logwarn("remote call failed: '" + node + "'")

                process_info[pid] = [node, hostname]

            except rosnode.ROSNodeIOException as error:
                rospy.logerr(error)
            except IOError as error:
                rospy.logerr(error)

        return process_info

    def run(self, frequency=1.0):

        rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            nodes_info = self.get_node_names_and_pids()
            nodes_info_array = NodeInfoArray()
            nodes_info_array.header = Header(stamp=rospy.Time.now())

            process_info = self.get_cpu_and_mem_usage(nodes_info.keys())

            for pid, [node, hostname] in nodes_info.iteritems():
                try:
                    nodes_info_array.data.append(NodeInfo(name=node, pid=pid, hostname=hostname,
                                                          cpu=process_info[pid][0], memory=process_info[pid][1],
                                                          error=NodeInfo.NO_ERROR))
                except KeyError:
                    rospy.logerr("error with PID of node '" + node + "'")
                    nodes_info_array.data.append(NodeInfo(name=node, pid=0, hostname='0.0.0.0',
                                                          cpu=0.0, memory=0,
                                                          error=NodeInfo.ERROR_PID_NOT_FOUND))

            self.node_infos_pub.publish(nodes_info_array)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('tug_resource_monitor_' + str(os.getpid()), anonymous=False)

    try:
        rospy.loginfo("starting " + rospy.get_name())
        TUGResourceMonitor().run(1)
    except KeyboardInterrupt:
        pass
    except rosnode.ROSNodeIOException as e:
        print e
    except rospy.ROSInterruptException:
        pass
