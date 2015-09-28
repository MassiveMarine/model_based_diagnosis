//
// Created by clemens on 03.08.15.
//

#ifndef TUG_OBSERVERS_CPP_OBSERVERPLUGINBASE_H
#define TUG_OBSERVERS_CPP_OBSERVERPLUGINBASE_H

#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <ros/ros.h>
#include <tug_observers/SubscriberFacade.h>
#include <tug_observers/Observation.h>

namespace tug_observers
{

    class ObserverPluginBase : public SubscriberFacade
    {
    private:
      ros::CallbackQueue internal_call_back_queue_;
      ros::AsyncSpinner spinner_;
      std::string type_;

    public:
      virtual ~ObserverPluginBase();

      void startPlugin();

      virtual void initialize(XmlRpc::XmlRpcValue params) = 0;

      void reportError(std::string resource, std::string error_msg, std::string verbose_error_msg, int32_t error_code, ros::Time time_of_occurence);

      void reportStates(std::string resource, std::vector<Observation> observations, ros::Time time_of_occurence);

      void flush();
    protected:
      ObserverPluginBase(std::string type);
    };
}

#endif //TUG_OBSERVERS_CPP_OBSERVERPLUGINBASE_H
