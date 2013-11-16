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
#import matplotlib.pyplot as plt
import tf
from geometry_msgs.msg import Quaternion
from std_msgs.msg._Header import Header
from rosgraph_msgs.msg._Log import Log
from tf.msg._tfMessage import tfMessage

class ColumnsRPY(object):
	def __init__(self, topic,lstname):
		self.topic = topic
		self.name = lstname
		self.time = []
		self.roll = []
		self.pitch = []
		self.yaw = []
		self.trpy_list= []
	def get_topic_name(self):
		return self.topic
	def get_name(self):
		return self.name
	def set_RPY(self,time,roll,pitch,yaw):
		self.trpy_list.append((time,roll,pitch,yaw))
	def get_RPY(self):
		return self.trpy_list
	def get_time(self):
		return self.time
	def get_roll(self):
		return self.roll
	def get_pitch(self):
		return self.pitch
	def get_yaw(self):
		return self.yaw

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
		if n == 0:
			return 0
		print self.node_name, self.cpu_list
		if n== 0:
			mean = 0
			sd = 0
		else:
			mean = sum(self.cpu_list)/n
			dev = [x - mean for x in self.cpu_list]
			dev2 = [x*x for x in dev]
			sd = sqrt( sum(dev2) / n)
		return mean+3*sd
	def set_mem(self,mem):
		self.mem_list.append(mem)
	def get_mem_list(self):
		return self.mem_list
	def get_mem(self):
		n = len(self.mem_list)
		if n == 0:
			return 0
		print self.node_name, self.mem_list
		mean = sum(self.mem_list)/n
		dev = [x - mean for x in self.mem_list]
		dev2 = [x*x for x in dev]
		sd = sqrt( sum(dev2) / n)
		print 'MeamMem=',mean,',SDMem=',sd,',3*sd=',3*sd
		return mean+3*sd
	def set_node_pid(self,pid):
		self.node_pid = pid
	def get_node_pid(self):
		return self.node_pid

class topic_data_structure(object):
	def __init__(self,topic_name,ws):
		self.topic_name = topic_name
		self.current_time_list = []
		self.delta_time_list = []
		self.mean_occ_time = 0
		self.frq = 0
	def get_topic_name(self):
		return self.topic_name
	def save_current_time(self,t):
		self.current_time_list.append(t)
	def get_current_time_list(self):
		return self.current_time_list
	def get_mean_occ_time(self):
		return self.mean_occ_time
	def get_deviation(self):
		n = len(self.delta_time_list)
		if n == 0:
			return 0
		self.mean_occ_time = float(sum(self.delta_time_list))/n
		dev = sqrt(sum((x-self.mean_occ_time)**2 for x in self.delta_time_list)/n)
		return dev
	def process_data(self):
		if len(self.current_time_list) == 0:
			return 0
		for x in range(1, len(self.current_time_list)):
			pt = self.current_time_list[x-1]		
			t = self.current_time_list[x]		
			self.delta_time_list.append(t-pt)
	def print_frq_list(self):
		print 'FILE: for'+self.topic_name
		fl = open(self.topic_name[1:len(self.topic_name)]+".txt", "w")
		for f in self.frq_list:
			fl.write(str(f)+"\n")
			print f
		fl.close()

class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.param_gen_ws = rospy.get_param('~gen_ws', 10)
		self.param_g_ws = rospy.get_param('~gob_ws', 10)
		self.param_p_ws = rospy.get_param('~pob_ws', 10)
		self.c = rospy.get_param('~occ_c', 10)
		self.p_mismatch_th = rospy.get_param('~pob_mismatch_th', 5)
		self.brd_topic = rospy.get_param('~board_topic', '/board_measurments')
		self.nodes_list = []
		self.topics_list = []
		self.zero_frq_topic_list = []
		self.topic_data_structure = []
		self.node_data_structure = []
		self.node_topic_list = []
		self.topics_type_list = []
		self.counter = 0
		self.global_list = []
		self.mean_delta_t = 0
		self.param_dev = 0
		self.maxCpu = 0
		self.maxMem = 0
		self.prev_t = time.time()
		self.excluded1 = []
		self.excluded2 = []
		self.fields = []
		self.topic = ''
		self.ok_topics_list = []
		self.topic_has_value = False
		self.Columns_list = []
		self.ColumnsRPY_list = []
		#self.topic_delta_sigma_list = []
		self.all_nodes_names_list = []
		self.all_nodes_cpu_list = []
		self.all_nodes_mem_list = []
		self.topic_names_list = []
		self.counter = 0
		self.mapping = []
		self.dataMAT = [[]]
		self.dataFile = open('safdarFile', 'w')
		self.basic_data_types = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
		self.basic_NOT_data_types = ['bool','str','string','time','std_msgs/Header','byte', 'duration']
		self.array_data_types = ['string[','float32[','float64[','int8[','uint8[','int16[','uint16[','int32[','uint32[','int64[','uint64[']

	def start(self):
		signal.signal(signal.SIGINT, self.signal_handler)
		self.extract_nodes_topics()
		thread.start_new_thread(self.threaded_extract_nodes_topics, ())
		thread.start_new_thread(self.threaded_extract_mem_cpu, ())
		thread.start_new_thread(self.spin_thread, ())
		while True:
			if interrupted:
		     		break;
			time.sleep(1)

	def auto_correlate(self,tpc_ds):
		lst = tpc_ds.occur_list
		mean = numpy.mean(lst)
		lst = lst-mean
		autocorr = numpy.correlate(lst, lst, mode='full')
		#autocorr = numpy.xcorr(s1n,"unbiased")
		return autocorr
			
	def end(self):
		i = 0
		for col in self.ColumnsRPY_list:
			i = i + 1
			print i, col.get_name(),'ROLL:',len(col.get_roll())
			i = i + 1
			print i, col.get_name(),'PITCH:',len(col.get_pitch())
			i = i + 1
			print i, col.get_name(),'YAW:',len(col.get_yaw())

		for col in self.Columns_list:
			i = i + 1
			print i, col[1],'DATA:',len(col[2])
			
        def signal_handler(self,signum, frame):
		global interrupted
		interrupted = True
		START_TIME = time.time()
		print 'START Time='+str(START_TIME)
		time.sleep(2)
		print 'making observers started....'
		self.make_obs_launch()
		print 'making observers finished....'
		print 'making model started....'
		self.make_mdl_yaml()
		print 'making model finished....'
		print 'matlabfile....'
		self.make_mat()
		END_TIME = time.time()
		self.end()
		print 'END Time='+str(END_TIME)
		print 'Total Calculated Time='+str(END_TIME - START_TIME)
		#self.topic_signal_nature()

	def make_mat(self):
		list_of_tdata = []
		list_rpy_data = []
		rpy_list_names = []
		rpy_topic_names = []
		rpy_names = []
		list_col_data = []
		list_dev_segma = []
		col_list_names = []
		col_topic_names = []
		col_names = []

		for tpc_ds in self.topic_data_structure:
			list_of_tdata.append(tpc_ds.get_current_time_list())

		for col in self.ColumnsRPY_list:
			rpy_topic_names.append(col.get_topic_name())
			rpy_list_names.append(col.get_name())
			list_rpy_data.append(col.get_RPY())
			
		for col in self.Columns_list:
			col_topic_names.append(col[0])
			col_list_names.append(col[1])
			list_col_data.append(col[2])
		
			
		mat_file = {'all_topic_names': self.topic_names_list, 'all_topic_occ_times': list_of_tdata, 'rpy_topic_names': rpy_topic_names, 'rpy_fileds_names': rpy_list_names,'rpy_fileds_data': list_rpy_data, 'col_topic_names': col_topic_names, 'col_fields_names': col_list_names,'col_fields_data': list_col_data, 'all_nodes_names': self.all_nodes_names_list, 'all_nodes_cpu': self.all_nodes_cpu_list, 'all_nodes_mem': self.all_nodes_mem_list}
		sio.savemat('/home/tedusar/code/fuerte_ws/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/generator.mat', {'model': mat_file})
		#sio.savemat('/home/safdar/fuerte_workspace/model_based_diagnosis/tug_ist_diagnosis_generator/matlab/generator.mat', {'model': mat_file})
		
       
	def spin_thread(self):
		rospy.spin()

	def topic_signal_nature(self):
		for tpc_ds in self.topic_data_structure:
			tpc_ds.get_signal_nature()

	def callback(self,data,topic):
		try:
			curr_t = rospy.get_rostime()
			time = curr_t.secs+curr_t.nsecs*1e-9
			for tpc_ds in self.topic_data_structure:
				if topic == tpc_ds.get_topic_name():
					tpc_ds.save_current_time(time)
			self.prev_t = curr_t
			if topic in self.ok_topics_list:
				fields = []
				self.rec_call(data,type(data),topic,fields,time)
		except AttributeError:
			e = sys.exc_info()[0]	
			print e			

	def rec_call(self,data,data_type,field,fields,time):
		fields.append(field)
		try:
			if type(data) == Quaternion:
				list_name = ''
				l = len(fields)
				for f in fields:
					list_name = list_name + f + '_'
				rpy = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])
				found =  False
				for col in self.ColumnsRPY_list:
					if col.get_name() == list_name:
						found = True
						col.set_RPY(time, float(rpy[0]), float(rpy[1]), float(rpy[2]))
						break
				if not found:
					obj = ColumnsRPY(fields[0],list_name)
					obj.set_RPY(time, float(rpy[0]), float(rpy[1]), float(rpy[2]))
					self.ColumnsRPY_list.append(obj)
				fields.pop()

			else:
				if (data_type in self.basic_data_types):
					list_name = ''
					l = len(fields)
					for f in fields:
						list_name = list_name + f + '_'
					found = False
					for col in self.Columns_list:
						if col[1] == list_name:
							col[2].append((time,data))
							found = True
							break
					if not found:
						lst = []
						lst.append((time,data))
						self.Columns_list.append((fields[0], list_name, lst))
					fields.pop()
				else:	
					data_type_str = str(data_type)
					j = data_type_str.find(']')
					if (data_type not in self.basic_NOT_data_types) & (j < 0):
							flds = data.__slots__
							x = 0
							for typ in data._slot_types:
								val = getattr(data,flds[x])
								self.rec_call(val,typ,flds[x],fields,time)
								x = x + 1
							fields.pop()
					else:
						fields.pop()
		except:
			pass

	def threaded_extract_mem_cpu(self):
		while True :
			for node_ds in self.node_data_structure:
				node_name = node_ds.get_node_name()
				node_pid = node_ds.get_node_pid()
				p = subprocess.Popen("top -b -n 1 | grep -i %s" %node_pid, shell=True,stdout=subprocess.PIPE)
				self.out = p.communicate()[0]
				self.out1 = shlex.split(self.out)
				cpu = self.out1[8]
				print 'node_name=',node_name, 'node_pid', node_pid
				print 'cpu=',cpu
				node_ds.set_cpu(float(cpu))
				p = subprocess.Popen("pmap -x %s" %node_pid, shell=True,stdout=subprocess.PIPE)
				out = p.communicate()[0]
				out1 = shlex.split(out)
				print 'mem=',out1[len(out1)-3]
				mem = float(out1[len(out1)-3])
				node_ds.set_mem(mem)
			time.sleep(0.25)

	def get_fieldsRecursive(self,field,msg_class):
		self.fields.append(field)
		#if msg_class.find('Quaternion') > -1:
		#	list_name = ''
		#	for f in self.fields:
		#		list_name = list_name + f + '_'
			#self.ColumnsRPY_list.append((list_name, ColumnsRPY(list_name)))		
		#t = ''
		#ln = len(msg_class)
		#msg_class_arr=''
		#if msg_class[ln-1] == ']':
			#j = msg_class.find('[')
			#t = msg_class[j+1:ln-1]
			#msg_class_arr = msg_class
			#msg_class = msg_class[0:j+1]
		#	print 'ARRAY:', msg_class
		#	self.fields.pop()
		if (msg_class in self.basic_data_types):
			list_name = ''
			for f in self.fields:
				list_name = list_name + f + '_'
			self.counter = self.counter + 1
			self.ok_topics_list.append(self.topic)
			print self.counter, ':', list_name , ' type:',msg_class
			#self.Columns_list.append((list_name, []))
			self.fields.pop()
		else:
			if (msg_class not in self.basic_NOT_data_types) & (msg_class[len(msg_class)-1] != ']'): #(msg_class not in self.array_data_types):
				#if msg_class[len(msg_class)-1] == '[':
				#	msg_class = msg_class[0:len(msg_class)-1]
				#print '1',msg_class
				msg_class = roslib.message.get_message_class(msg_class)
				#print msg_class
				fields = msg_class.__slots__
				x = 0
				for itm in msg_class._slot_types:
					self.get_fieldsRecursive(fields[x],itm)
					x = x + 1
				self.fields.pop()
			else:
				#if msg_class not in self.excluded1:
				#	self.excluded1.append(msg_class)
				#self.excluded2.append(self.fields[0]+':'+msg_class)
				#print self.topic,msg_class
				self.fields.pop()

	def extract_nodes_topics(self):
		#pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		#for topic in topicList:
		#	if topic[0] not in self.topics_list:
		#		self.topics_list.append(topic[0])
		#		msg_class = roslib.message.get_message_class(topic[1])
		#		self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
		#		rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		l = len(topicList)
		for i in xrange(l):
			self.topic = topicList[i][0]
			topic_type = topicList[i][1]
			self.topics_list.append(self.topic)
			#print '--------',self.topic,i,'---------'
			msg_class = roslib.message.get_message_class(topic_type)
			self.topic_data_structure.append(topic_data_structure(self.topic, self.param_gen_ws))
			self.get_fieldsRecursive(self.topic, topic_type)

		#for ex1 in self.excluded1:
		#	print 'Excluded:',ex1
		#for ex in self.excluded2:
		#	print ex

		#for obj in self.Columns_list:
		#	print 'OBJ: name=',obj[0],' class:', obj[1]
		#for obj in self.ColumnsRPY_list:
		#	print 'OBJRPY: name=',obj[0],' class:', obj[1]
		#ind = self.ColumnsRPY_list.index('/tf_transforms_transform_rotation_')
		#print 'INDEXXX', ind, 
		
		#print 'call back started ...'
		#rospy.spin()	
		#for n in self.nodes_list:
		#	print n
		#for t in self.topics_list:
		#	print t
		#print '==================='

		code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
		Pubs = sysState[0]
		Subs = sysState[1]
		for lst in sysState:
			for row in lst:
				for node in row[1]:
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
		for topic in topicList:
			#if (topic[0] in self.ok_topics_list) & (topic[0].find('/tf')>-1):
			if topic[0] in self.ok_topics_list:
				msg_class = roslib.message.get_message_class(topic[1])
				rospy.Subscriber(topic[0], msg_class, self.callback, topic[0], 100)
		print 'callback started...'
		#rospy.spin()



	def threaded_extract_nodes_topics(self):
		while True :
			pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
			for topic in topicList:
				if topic[0] not in self.topics_list:
					self.topics_list.append(topic[0])
					msg_class = roslib.message.get_message_class(topic[1])
					self.topic_data_structure.append(topic_data_structure(topic[0],self.param_gen_ws))
					rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		
			code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
			Pubs = sysState[0]
			Subs = sysState[1]
			for lst in sysState:
				for row in lst:
					for node in row[1]:
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

			time.sleep(0.5)
			
	def make_obs_launch(self):
		#obs_file_path = "/home/safdar/fuerte_workspace/model_based_diagnosis/tug_ist_diagnosis_launch/launch/obs_auto.launch"
		obs_file_path = "/home/tedusar/code/fuerte_ws/model_based_diagnosis/tug_ist_diagnosis_launch/launch/obs_auto.launch"
		file = open(bs_file_path, 'wb')
		file.write('<?xml version="1.0"?>\n<launch>\n')
		if self.brd_topic in self.topics_list:
			file.write('<node pkg="tug_ist_diagnosis_observers" type="HObs.py" name="BoardHObs" >\n')
			file.write('</node>\n')
		N = 'Nodes:'
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
			N = N+'('+node_name+', CPU='+str(node_mxCpu)+', MEM='+str(node_mxMem)+')'
			self.all_nodes_names_list.append(node_name)
			self.all_nodes_mem_list.append(nd_ds.get_mem_list())
			self.all_nodes_cpu_list.append(nd_ds.get_cpu_list())
		

		for t in self.topics_list:
			for tpc_ds in self.topic_data_structure:
				if t == tpc_ds.get_topic_name():
					tpc_ds.process_data()
					self.param_dev = tpc_ds.get_deviation()
					self.mean_delta_t = tpc_ds.get_mean_occ_time()
					self.param_g_ws = self.mean_delta_t
					self.param_g_ws = self.c * self.mean_delta_t
					N = N+'('+t+',MeanT='+str(self.mean_delta_t)+',Dev='+str(self.param_dev)+')'
					break
			topic_name = t[1:len(t)]
			gobs_name = topic_name+'GObs'
			file.write('<node pkg="tug_ist_diagnosis_observers" type="GObs.py" name="'+gobs_name+'" >\n')
			file.write('\t<param name="topic" value="'+topic_name+'" />\n')
  			file.write('\t<param name="frq" value="'+str(self.mean_delta_t)+'" />\n')
			file.write('\t<param name="dev" value="'+str(self.param_dev)+'" />\n')
  			file.write('\t<param name="ws" value="'+str(self.param_g_ws)+'" />\n')
  			file.write('</node>\n')
			self.topic_names_list.append(t)
			#self.topic_delta_sigma_list.append((self.mean_delta_t, self.param_dev))

		file.write('</launch>')
		file.close()

	def make_mdl_yaml(self):
		#model_file_path = "/home/safdar/fuerte_workspace/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		model_file_path = "/home/tedusar/code/fuerte_ws/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		file = open(model_file_path, 'wb')
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
		param = "_model:=/home/tedusar/code/fuerte_ws/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		#param = "_model:=/home/safdar/fuerte_workspace/my_workspace/model_based_diagnosis/tug_ist_diagnosis_model/diagnosis_model.yaml"
		output = subprocess.Popen(command+param , shell=True,stdout=subprocess.PIPE)
	
	def report_error(self):
		print '\n No Running System!'
		sys.exit(os.EX_USAGE)
			
if __name__ == '__main__':
	generator = Generator()
	generator.start()
	#generator.end()
