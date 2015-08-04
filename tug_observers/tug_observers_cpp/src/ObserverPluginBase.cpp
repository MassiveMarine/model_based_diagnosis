//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverPluginBase.h>

namespace tug_observers_cpp
{

    ObserverPluginBase::ObserverPluginBase() : spinner_(1, &internal_call_back_queue_)
    { }

    ObserverPluginBase::~ObserverPluginBase()
    {
      spinner_.stop();
    }

    void ObserverPluginBase::startPlugin()
    {
      spinner_.start();
    }

}