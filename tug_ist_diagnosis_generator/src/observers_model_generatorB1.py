#!/usr/bin/env python

##
# observers_generator.py is a node that finds the nodes and topics and their related information in the running system.
# After finding necessary information it creates launch files in tug_ist_diagnosis_launch package and executes them automatically. 
# Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

import roslib.message;roslib.load_manifest('tug_ist_diagnosis_generator')
import rospy
import sys
import xmlrpclib
import os
import subprocess
import time
import shlex
import thread
from math import sqrt
class node_data_structure(object):
	def __init__(self,node_name):
		self.node_name = node_name
		self.pub_topic_list = []
		self.sub_topic_list = []
		self.max_cpu = 0
		self.max_mem = 0
	def get_node_name(self):
		return self.node_name
	def add_pub_topic(self,topic):
		self.pub_topic_list.append(topic)
	def add_sub_topic(self,topic):
		self.sub_topic_list.append(topic)
	def get_pub_topics(self):
		return self.pub_topic_list
	def get_sub_topics(self):
		return self.sub_topic_list
	def set_max_cpu(self,max_cpu):
		self.max_cpu = max_cpu
	def get_max_cpu(self):
		return self.max_cpu
	def set_max_mem(self,max_mem):
		self.max_mem = max_mem
	def get_max_mem(self):
		return self.max_mem


class topic_data_structure(object):
	def __init__(self,topic_name,ws):
		self.topic_name = topic_name
		self.ws = ws
		self.circular_queu = [0 for i in xrange(self.ws)]
		self.frq_list = [0 for i in xrange(self.ws)]
		self.calculated_frq = 0
		self.calculated_frq_dev = 0
		self.prev_t = time.time()
	def get_topic_name(self):
		return self.topic_name
	def set_prev_t(self,prev_t):
		self.prev_t = prev_t
	def get_prev_t(self):
		return self.prev_t
	def calc_frq(self,delta_t):
		self.circular_queu.pop(0)
		self.circular_queu.append(delta_t)
		sm = 0
		for dt in self.circular_queu:
			sm = sm + dt
		self.calculated_frq = 1/(sm/self.ws)
		self.frq_list.pop(0)
        	self.frq_list.append(self.calculated_frq)
	def calculate_frq_dev(self):
		n = len(self.frq_list)
		mean = sum(self.frq_list)/n
		self.calculated_frq_dev = sqrt(sum((x-mean)**2 for x in self.frq_list)/n)	
	def get_calculated_frq(self):
		return self.calculated_frq
	def get_calculated_frq_dev(self):
		return self.calculated_frq_dev
	def print_frq_list(self):
		for f in self.frq_list:
			print f
		

class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.param_gen_ws = rospy.get_param('~gen_ws', 10)
		self.param_g_ws = rospy.get_param('~gob_ws', 10)
		self.param_p_ws = rospy.get_param('~pob_ws', 10)
		self.p_mismatch_th = rospy.get_param('~pob_mismatch_th', 5)
		self.brd_topic = rospy.get_param('~board_topic', '/board_measurments')
		self.nodes_list = []
		self.topics_list = []
		self.topic_data_structure = []
		self.node_data_structure = []
		self.node_topic_list = []
		self.topics_type_list = []
		self.counter = 0
		self.global_list = []
		self.param_frq = 0
		self.param_dev = 0
		

	def start(self):
		self.extract_topics()
		self.extract_nodes()
		self.make_obs_launch()
		self.make_mdl_yaml()
			
	def make_obs_launch(self):
		temp_path = "/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_launch/launch/obs_auto.launch"
		file = open(temp_path, 'wb')
		file.write('<?xml version="1.0"?>\n<launch>\n')
		if self.brd_topic in self.topics_list:
			file.write('<node pkg="tug_ist_diagnosis_observers" type="HObs.py" name="BoardHObs" >\n')
			file.write('</node>\n')
		for nd_ds in self.node_data_structure:
			node_name = nd_ds.get_node_name()
			node_name = node_name[1:len(node_name)]
			node_mxCpu = nd_ds.get_max_cpu()
			node_mxMem = nd_ds.get_max_mem()
			nobs_name = node_name+'NObs'
			pubs = nd_ds.get_pub_topics()
			if '/diagnostics' in pubs:
				dobs_name = node_name+'DObs'
				file.write('<node pkg="tug_ist_diagnosis_observers" type="DObs.py" name="'+dobs_name+'" >\n')
				file.write('\t<param name="dev_node" value="'+node_name+'" />\n')
				file.write('</node>\n')
			file.write('<node pkg="tug_ist_diagnosis_observers" type="NObs.py" name="'+nobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('</node>\n')
			pobs_name = node_name+'CpuPObs'	
			file.write('<node pkg="tug_ist_diagnosis_observers" type="PObs.py" name="'+pobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('\t<param name="property" value="cpu" />\n')
			file.write('\t<param name="max_val" value="'+node_mxCpu+'" />\n')
			file.write('\t<param name="mismatch_th" value="'+str(self.p_mismatch_th)+'" />\n')
			file.write('\t<param name="ws" value="'+str(self.param_p_ws)+'" />\n')
			file.write('</node>\n')
			pobs_name = node_name+'MemPObs'	
			file.write('<node pkg="tug_ist_diagnosis_observers" type="PObs.py" name="'+pobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('\t<param name="property" value="mem" />\n')
			file.write('\t<param name="max_val" value="'+node_mxMem+'" />\n')
			file.write('\t<param name="mismatch_th" value="'+str(self.p_mismatch_th)+'" />\n')
			file.write('\t<param name="ws" value="'+str(self.param_p_ws)+'" />\n')
			file.write('</node>\n')
		for t in self.topics_list:
			for tpc_ds in self.topic_data_structure:
				if t == tpc_ds.get_topic_name():
					tpc_ds.calculate_frq_dev()
					self.param_frq = tpc_ds.get_calculated_frq()
					self.param_dev = tpc_ds.get_calculated_frq_dev()
					break
			topic_name = t[1:len(t)]
			gobs_name = topic_name+'GObs'
			file.write('<node pkg="tug_ist_diagnosis_observers" type="GObs.py" name="'+gobs_name+'" >\n')
			file.write('\t<param name="topic" value="'+topic_name+'" />\n')
  			file.write('\t<param name="frq" value="'+str(self.param_frq)+'" />\n')
			file.write('\t<param name="dev" value="'+str(self.param_dev)+'" />\n')
  			file.write('\t<param name="ws" value="'+str(self.param_g_ws)+'" />\n')
  			file.write('</node>\n')
		file.write('</launch>')
		file.close()

	def make_mdl_yaml(self):
		temp_path = "/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		file = open(temp_path, 'wb')
		file.write('ab: "AB"\nnab: "NAB"\nneg_prefix: "not_"\n\nprops:\n')
		for n in self.nodes_list:
			node_name = n[1:len(n)]
			file.write('  - running('+node_name+')\n')
		for t in self.topics_list:
			topic_name = t[1:len(t)]
			file.write('  - ok('+topic_name+')\n')
		file.write('rules:\n')
		for nd_ds in self.node_data_structure:
			node_name = nd_ds.get_node_name()
			node_name = node_name[1:len(node_name)]
			file.write('  - NAB('+node_name+') -> running('+node_name+')\n')
			pubs = nd_ds.get_pub_topics()
			subs = nd_ds.get_sub_topics()
			for p in pubs:
				str_subs = ''
				for s in subs:
					str_subs = str_subs + ', ok('+ s[1:len(s)] +')'
				file.write('  - NAB('+node_name+')'+str_subs+' -> ok('+p[1:len(p)]+')\n')
		file.close()
	def extract_nodes(self):
		code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
		for lst in sysState:
			for row in lst:
				for node in row[1]:
					if (node == '/rosout') | (node == '/observers_generator') | (node == '/tf') | (node == '/board_controller') :
						continue
					if node not in self.nodes_list:
						self.nodes_list.append(node)
					
		i = 0
		for node in self.nodes_list:
			self.node_data_structure.append(node_data_structure(node))
			node_ds = self.node_data_structure[i]
			self.extract_mem_cpu(node,node_ds)
			for item in sysState[0]:
				if (node in item[1]) & (item[0] in self.topics_list):
					node_ds.add_pub_topic(item[0])
			for item in sysState[1]:
				if (node in item[1]) & (item[0] in self.topics_list):
					node_ds.add_sub_topic(item[0])
			i = i + 1
		
	def extract_mem_cpu(self, node, node_ds):
		a = subprocess.Popen("rosnode info " + node , shell=True,stdout=subprocess.PIPE)
		parts = shlex.split(a.communicate()[0])
		indx = parts.index("Pid:")
		pid = parts[indx+1]
		t = 0
		maxMem = 0
		maxCpu = 0
		while t<=6:
			p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
			self.out = p.communicate()[0]
			self.out1 = shlex.split(self.out)
			cpu = self.out1[8]
			mem = self.out1[9]
			if cpu > maxCpu:
				maxCpu = cpu
			if mem > maxMem:
				maxMem = mem
			t = t + 1
		node_ds.set_max_cpu(maxCpu)
		node_ds.set_max_mem(maxMem)
		
	def extract_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		i = 0
		for topic in topicList:
			if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg') | (topic[0] == '/tf') | (topic[0] == '/map_metadata') | (topic[0] == '/map') | ('/board_controller' in topic[0]) :
				continue
			if topic[0] not in self.topics_list:
				self.topics_list.append(topic[0])
			print 'topic 0', topic[0],'  topic 1', topic[1]
			msg_class = roslib.message.get_message_class(topic[1])
			self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
			sb=rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
			i = i + 1
		thread.start_new_thread(self.spin_thread,('thread', 1))
		time.sleep(3)
		
	def callback(self,data,topic):
		curr_t = time.time()
		for tpc_ds in self.topic_data_structure:
			if topic == tpc_ds.get_topic_name():
				delta_t = curr_t - tpc_ds.get_prev_t()
				tpc_ds.calc_frq(delta_t)
				tpc_ds.set_prev_t(curr_t)
				break
		
	def average_delta_t(self):
		s = 0
		for val in self.circular_queu:
			s = s + val
        	return s/self.param_ws

	def run_observers(self):
		output = subprocess.Popen("roslaunch tug_ist_diagnosis_launch obs_auto.launch" , shell=True,stdout=subprocess.PIPE)

	def run_model_server(self):
		command = "rosrun tug_ist_diagnosis_model diagnosis_model_server.py"
		param = "_model:=/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		output = subprocess.Popen(command+param , shell=True,stdout=subprocess.PIPE)
		
	
	def report_error(self):
		print '\n No Running System!'
		self.calculate_frq_dev()
		sys.exit(os.EX_USAGE)
	def spin_thread(self,string,st,*args):
		rospy.spin()
			
        
if __name__ == '__main__':
      generator = Generator()
      generator.start()
