#! /usr/bin/env python
## This code is made by Safdar Zaman on 20.1.2011 2:17pm Thursday Robotics Lab CityTower TU Graz Austria
## This Code is to find out the Published and Subscribed Topics.
## It also mentions the messages on the topic
## This program acts just like 'rostopic list -v' ros command on the terminal
## first import necessary lib packages
import os
import xmlrpclib
## catch the ID for the system
caller_id = '/script'
## get connected to the ROS MASTER
m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
## get State of the running ROS system into 3 variables
code, msg, val = m.getSystemState(caller_id)
## get info about all published topics and their messages
pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
## check for the successful connection
if code == 1:
  pubs, subs, srvs = val
else:
  print "call failed", code, msg
## get work started and play with the system variables now :-)
print "\nPublished topics:"
for pb in pubs:
   for t in topicList:
     if t[0] == pb[0]:
       if len(pb[1]) < 2:
         print " *", pb[0], "[",t[1],"]", len(pb[1]), " publisher"
       else:
         print " *", pb[0], "[",t[1],"]", len(pb[1]), " publishers"
print "\nSubscribed topics:"
for sb in subs:
  for t in topicList:
     if t[0] == pb[0]:
       if len(sb[1]) < 2:
         print " *", sb[0], "[",t[1],"]", len(sb[1]), " subscriber"
       else:
         print " *", sb[0], "[",t[1],"]", len(sb[1]), " subscribers"
