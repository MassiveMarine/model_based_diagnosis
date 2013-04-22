#! /usr/bin/env python
import os
import xmlrpclib
import time

caller_id = '/script'
m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
print"\nOrigin_NODE--------TOPIC--------->Destination_NODE\n"
code, msg, val = m.getSystemState(caller_id)
pubs, subs, srvs = val
for pb in pubs:
  for sb in subs:
    if pb[0] == sb[0]:
      for pbb in pb[1]:
        for sbb in sb[1]:
          for t in topicList:
            if t[0] == pb[0]:
              print pbb,"----------", pb[0],"[",t[1],"]" ,"-------->", sbb
