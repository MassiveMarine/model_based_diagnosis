//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverNode.h>

namespace tug_observers_cpp
{
    ObserverNode::ObserverNode(ros::NodeHandle nh) : nh_(nh)
    { }

    void ObserverNode::initPlugins()
    {
      XmlRpc::XmlRpcValue params;
      nh_.getParam("", params);
    }

    void ObserverNode::startPlugins()
    {

    }
}
