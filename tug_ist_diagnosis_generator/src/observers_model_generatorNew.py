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
import scipy.io as sio
import rospy
import sys
import xmlrpclib
import os
import subprocess
import time
import shlex
import thread
from math import sqrt
import signal
interrupted = False
import numpy
import matplotlib.pyplot as plt
import tf

class node_data_structure(object):
	def __init__(self,node_name):
		self.node_name = node_name
		self.pub_topic_list = []
		self.sub_topic_list = []
		self.cpu_list = []
		self.mem_list = []
		self.max_cpu = 0
		self.max_mem = 0
		self.node_pid = 0
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
	def set_cpu(self,cpu):
		self.cpu_list.append(cpu)
	def get_cpu_list(self):
		return self.cpu_list
	def get_cpu(self):
		n = len(self.cpu_list)
		print 'cpu list len'+str(n)
		print self.cpu_list
		if n== 0:
			mean = 0
			sd = 0
		else:
			mean = sum(self.cpu_list)/n
			dev = [x - mean for x in self.cpu_list]
			dev2 = [x*x for x in dev]
			sd = sqrt( sum(dev2) / n)
		return mean+2*sd
	def set_mem(self,mem):
		self.mem_list.append(mem)
	def get_mem_list(self):
		return self.mem_list
	def get_mem(self):
		n = len(self.mem_list)
		mean = sum(self.mem_list)/n
		dev = [x - mean for x in self.mem_list]
		dev2 = [x*x for x in dev]
		sd = sqrt( sum(dev2) / n)
		return mean+2*sd
	def set_node_pid(self,pid):
		self.node_pid = pid
	def get_node_pid(self):
		return self.node_pid


class topic_data_structure(object):
	def __init__(self,topic_name,ws):
		self.topic_name = topic_name
		self.ws = ws
		self.ws1 = ws/1000
		self.circular_queu = [0 for i in xrange(self.ws)]
		self.orien_list = []		
		self.frq_list = []
		self.time_list = []
		self.occur_list = []
		self.roll_list = []
		self.pitch_list = []
		self.yaw_list = []
		self.calculated_frq = 0
		self.calculated_frq_dev = 0
		self.prev_t = time.time()
	def get_topic_name(self):
		return self.topic_name
	def set_prev_t(self,prev_t):
		self.prev_t = prev_t
	def set_orientation(self,xyzw):
		self.orien_list.append(xyzw)
	def calcRPY(self):
		for o in self.orien_list:
			rpy = tf.transformations.euler_from_quaternion(o)
			self.roll_list.append(rpy[0])
			self.pitch_list.append(rpy[1])
			self.yaw_list.append(rpy[2])
	def get_orientation_list(self):
		return self.orien_list
	def getRoll(self):
		return self.roll_list
	def getPitch(self):
		return self.pitch_list
	def getYaw(self):
		return self.yaw_list
	def get_prev_t(self):
		return self.prev_t
	def set_occur(self,val):
		self.occur_list.append(val)
	def set_time(self,tm):
		self.time_list.append(tm)
	def get_time_list(self):
		return self.time_list
	def calc_frq(self,delta_t):
		self.circular_queu.pop(0)
		self.circular_queu.append(delta_t)
		sm = numpy.sum(self.circular_queu)
		#sm = 0
		#for dt in self.circular_queu:
		#	sm = sm + dt
		frq = 1/(float(sm)/self.ws)
		self.frq_list.append(delta_t)
		self.frq_list.append(delta_t+self.ws)
		#self.frq_list.append(sm)
		#self.frq_list.append(frq)
		#print 'size===',len(self.frq_list)
	def calculate_frq_dev(self):
		frq_list = self.frq_list[self.ws:]
		n = len(frq_list)
		if n!=0:
			#mean = self.calculate_mean(frq_list)
			mean = numpy.mean(frq_list)
			#self.calculated_frq_dev = self.calculate_standard_deviation(frq_list)
			self.calculated_frq_dev = numpy.std(frq_list)
			self.calculated_frq = mean
		else:
			mean = 0
			self.calculated_frq_dev = 0
			self.calculated_frq = 0
	def calculate_mean(self,par_list):
		n = len(par_list)
		s = sum(par_list)
		return 	float(s)/n
	def calculate_standard_deviation(self,par_list):
		n = len(par_list)
		mean = self.calculate_mean(par_list)
		return float(sqrt(sum((x-mean)**2 for x in par_list))/n)
	def get_calculated_frq(self):
		return self.calculated_frq
	def get_calculated_frq_dev(self):
		return self.calculated_frq_dev
	def get_signal_nature(self):
		#signal_mean = self.calculate_mean(self.occur_list)
		signal_mean = numpy.mean(self.occur_list)
		#signal_dev =  self.calculate_standard_deviation(self.occur_list)
		signal_dev =  numpy.std(self.occur_list)
		print self.topic_name, signal_mean, signal_dev
	def print_frq_list(self):
		print 'FILE: for'+self.topic_name
		topic_name = self.topic_name.replace("/","_");
		fl = open(topic_name[1:len(topic_name)]+".txt", "w")
		i = 1
		s = 0.0
		for f in self.time_list:
			fl.write(str(f)+"\n")
			if i == 1:
				p = f
				i = 2
				continue
			d = f - p
			p = f
			fl.write(str(d)+"\n")
			s = s + float(d)
			i = i + 1
		av = s / i
		fl.write("s="+str(s)+"\n")
		fl.write("i="+str(i)+"\n")
		fl.write("Avg="+str(round(av,4))+"\n")
			#print f
		fl.close()
		
		

class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.param_gen_ws = rospy.get_param('~gen_ws', 100)
		self.param_g_ws = rospy.get_param('~gob_ws', 10)
		self.param_p_ws = rospy.get_param('~pob_ws', 10)
		self.p_mismatch_th = rospy.get_param('~pob_mismatch_th', 5)
		self.brd_topic = rospy.get_param('~board_topic', '/board_measurments')
		self.orn_topics_field = rospy.get_param('~topics', 100)
		self.nodes_list = []
		self.topics_list = []
		self.zero_frq_topic_list = []
		self.topic_data_structure = []
		self.node_data_structure = []
		self.node_topic_list = []
		self.topics_type_list = []
		self.counter = 0
		self.global_list = []
		self.param_frq = 0
		self.param_dev = 0
		self.maxCpu = 0
		self.maxMem = 0
		self.param_topics = []
		self.param_fields = []


	def orn_separate_topic_field(self):
		pose_pose_orientation = 100
		orientation = 200
		print self.orn_topics_field
		s = self.orn_topics_field[1:len(self.orn_topics_field)]
		l = []
		while '/' in s:
			i = s.index('/')
			s1 = s[0:i]
			l.append(s1)
			s = s[i+1:len(s)]
		l.append(s)
		for t in l:
			i = t.index('+')
			top = t[0:i];
			self.param_topics.append('/'+top)
			fld = t[i+1:len(t)]
			fld1 = fld.replace('.','_')
			print fld1
			print  eval(fld1)
			self.param_fields.append(fld)
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		i = 0
		for topic in topicList:
			for t in self.param_topics:
				if t == topic[0]:
					print t+' matches '+ topic[0]
					self.topics_list.append(topic[0])
					msg_class = roslib.message.get_message_class(topic[1])
					if topic[0] not in self.topic_data_structure:
						self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
					rospy.Subscriber(topic[0], msg_class,self.callback_orien,[topic[0],self.param_fields[i]],10)
					#rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
					i = i + 1
		#rospy.spin()

	def callback_orien(self,msg,args):
		curr_t = time.time()
		topic = args[0]
		field =  args[1]
		x = eval('msg.%s.x' %field)
		y = eval('msg.%s.y' %field)
		z = eval('msg.%s.z' %field)
		w = eval('msg.%s.w' %field)
		orn = [x,y,z,w]
		#print topic
		#for tpc_ds in self.topic_data_structure:
			#if topic == tpc_ds.get_topic_name():
				#tpc_ds.set_orientation(orn)
				#tpc_ds.set_time(curr_t)

		
	def start(self):
		signal.signal(signal.SIGINT, self.signal_handler)
		#self.orn_separate_topic_field()
		self.extract_nodes_topics()
		#thread.start_new_thread(self.threaded_extract_nodes_topics, ())
		#thread.start_new_thread(self.threaded_extract_mem_cpu, ())
		#thread.start_new_thread(self.spin_thread, ())
		while True:
			if interrupted:
		      		break;

	def auto_correlate(self,tpc_ds):
		lst = tpc_ds.occur_list
		mean = numpy.mean(lst)
		lst = lst-mean
		autocorr = numpy.correlate(lst, lst, mode='full')
		return autocorr
			
			
        def signal_handler(self,signum, frame):
		global interrupted
		START_TIME = time.time()
		print 'START Time='+str(START_TIME)
		interrupted = True
		time.sleep(1)
		print 'making matfile started....'
		#self.make_mat()
		#print 'making matfile finished....'
		#print 'making Nodes matfile started....'
		#self.make_node_mat()
		#print 'making Nodes matfile finished....'
		#self.make_obs_launch()
		#print 'making observers finished....'
		#print 'making model started....'
		#self.make_mdl_yaml()
		#print 'making model finished....'
		END_TIME = time.time()
		print 'END Time='+str(END_TIME)
		print 'Total Calculated Time='+str(END_TIME - START_TIME)
		#self.topic_signal_nature()
		

	def make_mat(self):
		list_of_tdata = []
		list_of_topics = []
		list_of_rpy = []
		for tpc_ds in self.topic_data_structure:
			top_name = tpc_ds.get_topic_name()
			list_of_tdata.append(tpc_ds.get_time_list())
			list_of_topics.append(top_name)
			if top_name in self.param_topics:
				tpc_ds.calcRPY()
				list_of_rpy.append(tpc_ds.getRoll())
				list_of_rpy.append(tpc_ds.getPitch())
				list_of_rpy.append(tpc_ds.getYaw())
			
		list_of_ndata = []
		list_of_nodes = []
		for nd_ds in self.node_data_structure:
			list_of_ndata.append(nd_ds.get_cpu_list())
			list_of_ndata.append(nd_ds.get_mem_list())
			list_of_nodes.append(nd_ds.get_node_name())
		#node_mat = {'nodes': list_of_nodes, 'nodes_time': list_of_ndata}
		#topic_mat = {'topics': list_of_topics, 'topics_time': list_of_tdata}
		mat_file = {'topics': list_of_topics, 'topics_time': list_of_tdata, 'topics_rpy': list_of_rpy,'nodes': list_of_nodes, 'nodes_cpu_mem': list_of_ndata}
		#sio.savemat('/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/topic_lists.mat', {'topics': topic_mat})
		sio.savemat('/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/model_mat.mat', {'model': mat_file})

	def make_node_mat(self):
		list_of_ndata = []
		list_of_nodes = []
		for nd_ds in self.node_data_structure:
			list_of_ndata.append(nd_ds.get_cpu_list())
			list_of_ndata.append(nd_ds.get_mem_list())
			list_of_nodes.append(nd_ds.get_node_name())
		node_mat = {'nodes': list_of_nodes, 'nodes_time': list_of_ndata}
		sio.savemat('/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/node_lists.mat', {'nodes': node_mat})

       
	def spin_thread(self):
		rospy.spin()

	def topic_signal_nature(self):
		for tpc_ds in self.topic_data_structure:
			tpc_ds.get_signal_nature()

	def callback(self,data,topic):
		print topic
		#curr_t = time.time()
		#for tpc_ds in self.topic_data_structure:
			#occurance = 0
			#if topic == tpc_ds.get_topic_name():
				#tpc_ds.set_time(curr_t)
				


	def threaded_extract_mem_cpu(self):
		while True :
			for node_ds in self.node_data_structure:
				node_name = node_ds.get_node_name()
				node_pid = node_ds.get_node_pid()
				p = subprocess.Popen("top -b -n 1 | grep -i %s" %node_pid, shell=True,stdout=subprocess.PIPE)
				self.out = p.communicate()[0]
				self.out1 = shlex.split(self.out)
				cpu = self.out1[8]
				mem = self.out1[9]
				#if cpu > self.maxCpu:
				#	self.maxCpu = cpu
				#if mem > self.maxMem:
				#	self.maxMem = mem
				node_ds.set_cpu(float(cpu))
				node_ds.set_mem(float(mem))
				#print 'node_name=',node_name,' node_pid=',node_pid, ' Max_cpu=',self.maxCpu,' Max_Mem=',self.maxMem
			time.sleep(0.25)

	def extract_nodes_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		#print topicList			
		for topic in topicList:
			#if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg'):
			#	continue
			if topic[0] not in self.topics_list:
				self.topics_list.append(topic[0])
				msg_class = roslib.message.get_message_class(topic[1])
				self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
				rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		for n in self.nodes_list:
			print n
		for t in self.topics_list:
			print t
		print '==================='

		code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
		#print sysState
		Pubs = sysState[0]
		Subs = sysState[1]
		for lst in sysState:
			for row in lst:
				for node in row[1]:
					#if (node == '/rosout') | (node == '/observers_generator') | ('rostopic' in node):
					if (node == '/observers_generator') | ('rostopic' in node) :
						continue
					if node not in self.nodes_list:
						self.nodes_list.append(node)
						a = subprocess.Popen("rosnode info " + node , shell=True,stdout=subprocess.PIPE)
						parts = shlex.split(a.communicate()[0])
						indx = parts.index("Pid:")
						pid = parts[indx+1]
						nd_ds = node_data_structure(node)
						nd_ds.set_node_pid(pid)
						for item in Pubs:
							if (node in item[1]) & (item[0] in self.topics_list):
								nd_ds.add_pub_topic(item[0])
						for item in Subs:
							if (node in item[1]) & (item[0] in self.topics_list):
								nd_ds.add_sub_topic(item[0])
						self.node_data_structure.append(nd_ds)
						print 'node=',node,' pid=',pid
					
	def threaded_extract_nodes_topics(self):
		while True :
			pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
			#print topicList			
			for topic in topicList:
				#if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg') :
				#	continue
				if topic[0] not in self.topics_list:
					self.topics_list.append(topic[0])
					msg_class = roslib.message.get_message_class(topic[1])
					self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
					rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])

			#print '------------------------'
	
			code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
			#print sysState
			Pubs = sysState[0]
			Subs = sysState[1]
			for lst in sysState:
				for row in lst:
					for node in row[1]:
						#if (node == '/rosout') | (node == '/observers_generator')  | ('rostopic' in node):
						if (node == '/observers_generator') | ('rostopic' in node) :
							continue
						if node not in self.nodes_list:
							self.nodes_list.append(node)
							
							a = subprocess.Popen("rosnode info " + node , shell=True,stdout=subprocess.PIPE)
							parts = shlex.split(a.communicate()[0])
							indx = parts.index("Pid:")
							pid = parts[indx+1]
							nd_ds = node_data_structure(node)
							nd_ds.set_node_pid(pid)
							for item in Pubs:
								if (node in item[1]) & (item[0] in self.topics_list):
									nd_ds.add_pub_topic(item[0])
							for item in Subs:
								if (node in item[1]) & (item[0] in self.topics_list):
									nd_ds.add_sub_topic(item[0])
							self.node_data_structure.append(nd_ds)
							#print 'node=',node,' pid=',pid

			time.sleep(0.5)
			
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
			node_mxCpu = nd_ds.get_cpu()
			node_mxMem = nd_ds.get_mem()
			nobs_name = node_name+'NObs'
			pubs = nd_ds.get_pub_topics()
			#if '/diagnostics' in pubs:
			#	dobs_name = node_name+'DObs'
			#	file.write('<node pkg="tug_ist_diagnosis_observers" type="DObs.py" name="'+dobs_name+'" >\n')
			#	file.write('\t<param name="dev_node" value="'+node_name+'" />\n')
			#	file.write('</node>\n')
			file.write('<node pkg="tug_ist_diagnosis_observers" type="NObs.py" name="'+nobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('</node>\n')
			pobs_name = node_name+'CpuPObs'	
			file.write('<node pkg="tug_ist_diagnosis_observers" type="PObs.py" name="'+pobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('\t<param name="property" value="cpu" />\n')
			file.write('\t<param name="max_val" value="'+str(node_mxCpu)+'" />\n')
			file.write('\t<param name="mismatch_th" value="'+str(self.p_mismatch_th)+'" />\n')
			file.write('\t<param name="ws" value="'+str(self.param_p_ws)+'" />\n')
			file.write('</node>\n')
			pobs_name = node_name+'MemPObs'	
			file.write('<node pkg="tug_ist_diagnosis_observers" type="PObs.py" name="'+pobs_name+'" >\n')
			file.write('\t<param name="node" value="'+node_name+'" />\n')
			file.write('\t<param name="property" value="mem" />\n')
			file.write('\t<param name="max_val" value="'+str(node_mxMem)+'" />\n')
			file.write('\t<param name="mismatch_th" value="'+str(self.p_mismatch_th)+'" />\n')
			file.write('\t<param name="ws" value="'+str(self.param_p_ws)+'" />\n')
			file.write('</node>\n')
		list_of_lists = []
		list_of_names = []
		for t in self.topics_list:
			for tpc_ds in self.topic_data_structure:
				if t == tpc_ds.get_topic_name():
					tpc_ds.calculate_frq_dev()
					#y = 'imu'
					#if y in t:
					tpc_ds.print_frq_list()
					list_of_lists.append(tpc_ds.get_time_list())
					list_of_names.append(tpc_ds.get_topic_name())
					self.param_frq = tpc_ds.get_calculated_frq()
					self.param_dev = tpc_ds.get_calculated_frq_dev()
					break
			if self.param_frq == 0:
				self.zero_frq_topic_list.append(t)
				continue
			topic_name = t[1:len(t)]
			gobs_name = topic_name+'GObs'
			file.write('<node pkg="tug_ist_diagnosis_observers" type="GObs.py" name="'+gobs_name+'" >\n')
			file.write('\t<param name="topic" value="'+topic_name+'" />\n')
  			file.write('\t<param name="frq" value="'+str(self.param_frq)+'" />\n')
			file.write('\t<param name="dev" value="'+str(2*self.param_dev)+'" />\n')
  			file.write('\t<param name="ws" value="'+str(self.param_g_ws)+'" />\n')
  			file.write('</node>\n')
		file.write('</launch>')
		file.close()
		mat_list = {'list_names': list_of_names, 'list_data': list_of_lists }
		sio.savemat('/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/time_lists.mat', {'time_lists': mat_list})


	def make_mdl_yaml(self):
		temp_path = "/home/safdar/my_workspace/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		file = open(temp_path, 'wb')
		file.write('ab: "AB"\nnab: "NAB"\nneg_prefix: "not_"\n\nprops:\n')
		for n in self.nodes_list:
			node_name = n[1:len(n)]
			file.write('  - running('+node_name+')\n')
			file.write('  - ok('+node_name+',cpu)\n')
			file.write('  - ok('+node_name+',mem)\n')
		for t in self.topics_list:
			if t in self.zero_frq_topic_list:
				continue
			topic_name = t[1:len(t)]
			file.write('  - ok('+topic_name+')\n')
		file.write('rules:\n')
		for nd_ds in self.node_data_structure:
			node_name = nd_ds.get_node_name()
			node_name = node_name[1:len(node_name)]
			file.write('  - NAB('+node_name+') -> running('+node_name+')\n')
			file.write('  - NAB('+node_name+'), running('+node_name+') -> ok('+node_name+',cpu)\n')
			file.write('  - NAB('+node_name+'), running('+node_name+') -> ok('+node_name+',mem)\n')
			pubs = nd_ds.get_pub_topics()
			subs = nd_ds.get_sub_topics()
			for p in pubs:
				if p in self.zero_frq_topic_list:
					continue
				str_subs = ''
				for s in subs:
					if s in self.zero_frq_topic_list:
						continue
					str_subs = str_subs + ', ok('+ s[1:len(s)] +')'
				file.write('  - NAB('+node_name+')'+str_subs+' -> ok('+p[1:len(p)]+')\n')
		file.close()
	def extract_nodes(self):
		code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
		for lst in sysState:
			for row in lst:
				for node in row[1]:
					#if (node == '/rosout') | (node == '/observers_generator') | (node == '/tf') | (node == '/board_controller') :
					if (node == '/observers_generator'):
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
			#if cpu > maxCpu:
			#	maxCpu = cpu
			#if mem > maxMem:
			#	maxMem = mem
			t = t + 1
			node_ds.set_cpu(float(cpu))
			node_ds.set_mem(float(mem))
		
	def extract_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		i = 0
		for topic in topicList:
			#if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg') | (topic[0] == '/tf') | (topic[0] == '/map_metadata') | (topic[0] == '/map') | ('/board_controller' in topic[0]) :
			#	continue
			if topic[0] not in self.topics_list:
				self.topics_list.append(topic[0])
			print 'topic 0', topic[0],'  topic 1', topic[1]
			msg_class = roslib.message.get_message_class(topic[1])
			self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
			sb=rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
			i = i + 1
		thread.start_new_thread(self.spin_thread,('thread', 1))
		time.sleep(1)
		
		
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
		sys.exit(os.EX_USAGE)
			
if __name__ == '__main__':
	generator = Generator()
	generator.start()
