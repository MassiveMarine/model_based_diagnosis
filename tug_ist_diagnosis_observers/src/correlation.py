#!/usr/bin/env python

##
# The Diagnostic Observer observes device status on the /diagnostics topic whether its OK, WARNING or ERROR.
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
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

import roslib.message; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
from std_msgs.msg import String
from numpy import arange,array,ones,linalg,cumsum,median
from pylab import plot,show
from scipy import stats
import scipy.io



class Correlation(object):
    def __init__(self):
	rospy.init_node('correlation_node', anonymous=False)
	self.t = [0,1,2,3,4,5,6,7,8,9]
	self.d = [10,11,12,13,14,15,16,17,18,19]
	self.queu_t = []
	self.queu_d = []
	self.Id  = 0
	self.ws  = 0
	self.cntrl_indx = 0
	self.curr_t = 0
	self.w_strt_t = 0
	self.w_end_t = 0
	self.left_time = -1000
	self.right_time = -1
	self.q_l_indx = -1
	self.q_r_indx = -1
	self.central_t = 0
         
    def start(self):

	xi = arange(0,9)
	A = array([ xi, ones(9)])
	# linearly generated sequence
	y = [19, 20, 20.5, 21.5, 22, 23, 23, 25.5, 24]
	w = linalg.lstsq(A.T,y)[0] # obtaining the parameters
	print 'x=',xi
	print 'y=',y
	print 'LG coff=',w
	# plotting the line
	line = w[0]*xi+w[1] # regression line
	plot(xi,line,'r-',xi,y,'o')
	show()

    def start2(self):

	x = arange(0,9)
	A = array([ x, ones(9)])
	# linearly generated sequence
	y = [19, 20, 20.5, 21.5, 22, 23, 23, 25.5, 24]

	slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)

	print 'x', x
	print  'y', y
	print 'slope', slope
	print 'intercept', intercept
	line = slope*x+intercept
	#plot(x,line,'r-',x,y,'o')
	#show()

    def linear_regression2(self,t,s,ws):
	Sum_xy = 0.0
	Sum_x = 0.0
	Sum_y = 0.0
	Sum_xx = 0.0
	last_indx =  len(t) - 1
	i = last_indx
	n = 0
	while (i >-1) & ( (t[last_indx]-t[i]) < ws ):
		Sum_xy = Sum_xy + t[i] * s[i]
		Sum_x = Sum_x + t[i]
		Sum_y = Sum_y + s[i]
		Sum_xx = Sum_xx + t[i] * t[i]
		i = i - 1
		n = n + 1
	self.n = n
	if (n * Sum_xx - (Sum_x * Sum_x)) <> 0 :
		slope = (n*Sum_xy - Sum_x * Sum_y)/(n * Sum_xx - (Sum_x * Sum_x))
		return slope
	else:
		return 0

    def linear_regression(self,t,s,ws):
	Sum_xy = 0.0
	Sum_x = 0.0
	Sum_y = 0.0
	Sum_xx = 0.0
	n = len(t)
	for i in xrange(n):
		Sum_xy = Sum_xy + t[i] * s[i]
		Sum_x = Sum_x + t[i]
		Sum_y = Sum_y + s[i]
		Sum_xx = Sum_xx + t[i] * t[i]
	slope = (n*Sum_xy - Sum_x * Sum_y)/(n * Sum_xx - (Sum_x * Sum_x))
	return slope

    def deriveStreamLib(self,t,d,w):
	derv = []
    	for i in range(0,len(t)):
    		curr_t = t[i]
    		strt_t = curr_t - w/2
    		last_t = curr_t + w/2
    		k = i
     		while t[k]>strt_t:
        		if k==0:
            			break
        		else:
            			k = k - 1
		w_strt_indx = k
     		w_end_indx = 0
    		for j in range(i,len(t)):
        		if (t[j] > last_t) or (j==len(t)-1): 
            			w_end_indx = j - 1
            			break;
    		x = t[w_strt_indx:w_end_indx+1]
    		y = d[w_strt_indx:w_end_indx+1]
    		slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
		#print x, y, slope
		#quit()
		#slope = self.linear_regression(x,y,w)
    		derv.append(slope)
	return derv

    def deriveStreamSaf(self,t,d,w):
	derv = []
    	for i in range(0,len(t)):
    		curr_t = t[i]
    		strt_t = curr_t - w/2
    		last_t = curr_t + w/2
    		k = i
     		while t[k]>strt_t:
        		if k==0:
            			break
        		else:
            			k = k - 1
		w_strt_indx = k
     		w_end_indx = 0
    		for j in range(i,len(t)):
        		if (t[j] > last_t) or (j==len(t)-1): 
            			w_end_indx = j - 1
            			break;
    		x = t[w_strt_indx:w_end_indx+1]
    		y = d[w_strt_indx:w_end_indx+1]
    		slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
		#slope = self.linear_regression(x,y,w)
    		derv.append(slope)
	return derv

    def deriveStream(self,t,d,w):
	derv = []
    	for i in range(0,len(t)):
    		curr_t = t[i]
    		strt_t = curr_t - w/2
    		last_t = curr_t + w/2
    		k = i
     		while t[k]>strt_t:
        		if k==0:
            			break
        		else:
            			k = k - 1
		w_strt_indx = k
     		w_end_indx = 0
    		for j in range(i,len(t)):
        		if (t[j] > last_t) or (j==len(t)-1): 
            			w_end_indx = j - 1
            			break;
    		x = t[w_strt_indx:w_end_indx+1]
    		y = d[w_strt_indx:w_end_indx+1]
    		slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
		#slope = self.linear_regression(x,y,w)
    		derv.append(slope)
	return derv

    def calculateR123(self,t,d,ws):
	R123 = [[],[],[]]
	r1 = self.deriveStream(t,d,ws)
	print 'r1 calculated'
	R123[0].append(r1)
	r2 = self.deriveStream(t,r1,ws)
	print 'r2 calculated'
	R123[1].append(r2)
	r3 = self.deriveStream(t,r2,ws)
	print 'r3 calculated'
	R123[2].append(r3)
	return R123

    def abstractStream(self, r1, r2, r3):
	pos_b = []
	neg_b = []
	pos_b.append([x for x in r1 if x > 0])
	neg_b.append([x for x in r1 if x < 0])
    	high_b = median(pos_b)
    	low_b = median(neg_b)
	Trends = []
    	Trends.append(0)
	print 'len r1=', len(r1[:])
     	for i in range(1,len(r1[:])):
		if r1[i] < low_b:
        	    Trends.append(-1);
        	else:
			if r1[i] > high_b:
	        	    Trends.append(1)
        		else:
				if not (((low_b<r2[i]) and (r2[i]<high_b)) and ((low_b<r3[i]) and (r3[i]<high_b))):
        			    	Trends.append(0)
        			else:
        	    			Trends.append(Trends[i-1])
	return Trends

    def start3(self):
	mat = scipy.io.loadmat('/home/safdar/correlation/DataForTestPy.mat')
	t = mat['t1o']
	t_ = mat['t2o']
	d1 = mat['d1']
	numRows = len(t[:])
	numCols = len(t[0][:])
	t1 = []
	for i in xrange(numRows):
		t1.append(t[i][0])
	if t_[0][0] < t1[0]:
		t1 = t1[:] - t_[0][0]
	else:
		t1 = t1[:] - t1[0]
	#print 'Rows = ', numRows, 'Cols=', numCols
	#print 'Val', t1, t1[0]
	Id1 = cumsum(d1[:])
	#print Id1[3]
	c = 10;
	ws1 = c * 0.0501792663  #106
	R123 = self.calculateR123(t1,Id1,ws1)
	#s = self.deriveStreamLib(t1, Id1, ws1)
	#print 'slope calculated'
	#R123m = array(R123, dtype=object)
	trnd1 = self.abstractStream(R123[0][0], R123[1][0], R123[2][0])
	scipy.io.savemat('/home/safdar/correlation/PyGenerated.mat', mdict={'Py_trnd1':trnd1})
	print 'DONE'


    def slopes(self):
	mat = scipy.io.loadmat('/home/safdar/correlation/DataForTestPy.mat')
	t1_ = mat['t1o']
	d1_ = mat['d1']
	
	t2_ = mat['t2o']
	d2_ = mat['d2']
	
	t3_ = mat['t3o']
	d3_ = mat['d3']

	ws1 = mat['ws1']
	ws2 = mat['ws2']
	ws3 = mat['ws3']


	numRows1 = len(t1_[:])
	numRowsd1 = len(d1_[:])

	numRows2 = len(t2_[:])
	numRowsd2 = len(d2_[:])

	numRows3 = len(t3_[:])
	numRowsd3 = len(d3_[:])
	
	t1 = []
	for i in xrange(numRows1):
		t1.append(t1_[i][0])
	d1 = []
	for i in xrange(numRowsd1):
		d1.append(d1_[i][0])
	t2 = []
	for i in xrange(numRows2):
		t2.append(t2_[i][0])
	d2 = []
	for i in xrange(numRowsd2):
		d2.append(d2_[i][0])
	t3 = []
	for i in xrange(numRows3):
		t3.append(t3_[i][0])
	d3 = []
	for i in xrange(numRowsd3):
		d3.append(d3_[i][0])

	#t1 = t1[:] - t1[0]
	#t2 = t2[:] - t2[0]
	#t3 = t3[:] - t3[0]
	
	#c = 10
	#ws1 = 0.200039527055020  
	#ws2 = 0.501411240687562 
	#ws3 = 0.500085202031820
	print 'ws1=',ws1[0][0]
	print 'ws2=',ws2[0][0]
	print 'ws3=',ws3[0][0]
	
	#quit()	

	slops1Lib = self.deriveStreamLib(t1, d1, ws1[0][0])
	print 'slopes1Lib Done'#, slops1Lib
	slops2Lib = self.deriveStreamLib(t2, d2, ws2[0][0])
	print 'slopes2Lib Done', len(t2),len(slops2Lib)
	slops3Lib = self.deriveStreamLib(t3, d3, ws3[0][0])
	print 'slopes3Lib Done'#, slops3Lib
	slops1Saf = self.deriveStreamSaf(t1, d1, ws1[0][0])
	print 'slopes1Saf Done'
	slops2Saf = self.deriveStreamSaf(t2, d2, ws2[0][0])
	print 'slopes2Saf Done'
	slops3Saf = self.deriveStreamSaf(t3, d3, ws3[0][0])
	print 'slopes3Saf Done' ,len(slops3Lib), len(slops3Saf[:])
	#k = 0
	#for i in xrange(len(slops3Saf[:])):
	#	if (slops3Lib[i] - slops3Saf[i]) != 0:
	#		print i, slops3Lib[i], slops3Saf[i]
	#
	#print slops1Lib[0], slops1Saf[0], slops2Lib[0], slops2Saf[0], slops3Lib[0], slops3Saf[0]	
	#print t1[0], d1[0], t2[0], d2[0], t3[0], d3[0]	
	#print k

	scipy.io.savemat('/home/safdar/correlation/PyGenerated.mat', mdict={'slopes1Lib':slops1Lib,'slopes2Lib':slops2Lib,'slopes3Lib':slops3Lib,'slopes1Saf':slops1Saf,'slopes2Saf':slops2Saf,'slopes3Saf':slops3Saf,'t1P':t1,'d1P':d1})
	print 'DONE'
	plot(t1,d1,'r')
	plot(t2,d2,'b')
	plot(t3,d3,'g')
	show()

    def callback_fake(self,t,d):
	self.Id = self.Id + d
	self.queu_t.append(t)
	self.queu_d.append(self.Id)
	self.find_trend(t)

    def window_items(self,indx1,indx2):
	j = 0
	for i in range(indx1,indx2+1):
		print i,
		print self.queu_t[i]
		j = j + 1
	print 'total = ',j

    def find_trend(self,curr_t):
	self.central_t = self.queu_t[self.cntrl_indx]
	self.left_time = self.central_t - self.ws/2
	self.right_time = self.central_t + self.ws/2


    def find_trend1(self,curr_t):
	self.central_t = self.queu_t[self.cntrl_indx]
	self.left_time = self.central_t - self.ws/2
	self.right_time = self.central_t + self.ws/2
	if curr_t > self.right_time:
		#print self.queu_t
		self.q_r_indx = len(self.queu_t) - 1
		while self.queu_t[self.q_r_indx] > self.right_time:
			self.q_r_indx = self.q_r_indx - 1
		print 'Currt',curr_t,'cen_t=',self.central_t,'lt=',self.left_time,'rt=',self.right_time,'length'
		self.window_items(0,self.q_r_indx)
		#print self.queu_t[0:self.q_r_indx+1]
		slope, intercept, r_value, p_value, std_err = stats.linregress(self.queu_t[0:self.q_r_indx+1],self.queu_d[0:self.q_r_indx+1])
		print 'slope=',slope
		self.cntrl_indx = self.cntrl_indx + 1
		self.central_t = self.queu_t[self.cntrl_indx]
		self.left_time = self.central_t - self.ws/2
		self.right_time = self.central_t + self.ws/2
	if self.left_time > self.queu_t[0]:
		self.queu_t.pop(0)
		self.queu_d.pop(0)
		self.cntrl_indx = self.cntrl_indx - 1
		if self.w_strt_t == 3:
			quit()
		else:
			self.w_strt_t = self.w_strt_t + 1
		
    def start4(self):
	mat = scipy.io.loadmat('/home/safdar/correlation/DataForTestPy.mat')
	t = mat['t1o']
	t_ = mat['t2o']
	d1 = mat['d1']
	numRows = len(t[:])
	numCols = len(t[0][:])
	t = t[:] - t[0][0]
	c = 10;
	self.ws = c * 0.0501792663  #106
 
	for i in xrange(numRows):
		self.callback_fake(t[i][0],d1[i][0])
	print 'DONE'


if __name__ == '__main__':
	cor = Correlation()
	#cor.start()
	#cor.start2()
	#cor.start3()
	#cor.start4()
	cor.slopes()
