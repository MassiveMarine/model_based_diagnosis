#!/usr/bin/env python

# basics
import rospy

import rosnode
from tug_resource_monitor.msg import NodeInfo, NodeInfoArray
import psutil

processes = dict()


def get_cpu_and_mem_usage(pids=[]):
    process_info = dict()

    for pid in pids:
        try:
            if not processes.has_key(pid):
                processes[pid] = psutil.Process(pid)

            memory_info = processes[pid].get_memory_info()[0]
            cpu_percent = processes[pid].get_cpu_percent(interval=0)

            process_info[pid] = [cpu_percent, memory_info]
        except psutil.NoSuchProcess, KeyError:
            pass

    # for proc in psutil.process_iter():
    #     try:
    #         if proc.pid in pids or not pids:
    #             memory_info = proc.get_memory_info()[0]
    #             cpu_percent = proc.get_cpu_percent(interval=0)
    #             pid = proc.pid
    #             process_info[pid] = [cpu_percent, memory_info]
    #
    #     except psutil.NoSuchProcess:
    #         pass

    return process_info


def get_node_names_and_pids():
    process_info = dict()

    node_names = rosnode.get_node_names()
    master = rosnode.rosgraph.Master(rosnode.ID)

    for node in node_names:
        node_api = rosnode.get_api_uri(master, node)
        if not node_api:
            print("cannot find [%s]: unknown node" % node)
            continue
        try:
            code, msg, pid = rosnode.ServerProxy(node_api).getPid(rosnode.ID)
            if code != 1:
                raise rosnode.ROSNodeException("remote call failed: %s" % msg)
        except:
            continue

        process_info[pid] = node
    return process_info


def run(frequency=1.0):

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():

        nodes_info_array = NodeInfoArray()

        nodes_info = get_node_names_and_pids()
        process_info = get_cpu_and_mem_usage(nodes_info.keys())

        for pid, node in nodes_info.iteritems():
            nodes_info_array.data.append(NodeInfo(name=node, pid=pid, cpu=process_info[pid][0], memory=process_info[pid][1]))

        node_infos_pub.publish(nodes_info_array)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('tug_resource_monitor', anonymous=False)

    node_infos_pub = rospy.Publisher('diag/node_infos', NodeInfoArray, queue_size=10)

    try:
        run(1)
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
