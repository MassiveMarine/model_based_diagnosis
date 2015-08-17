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
#include <ros/node_handle.h>

namespace tug_observers_cpp
{

    class ObserverPluginBase
    {
    private:
      ros::CallbackQueue internal_call_back_queue_;
      ros::AsyncSpinner spinner_;
      ros::NodeHandle nh_;
      ros::Publisher error_pub_;
      std::string type_;

    public:
      virtual ~ObserverPluginBase();

      void startPlugin();

      virtual void initialize(XmlRpc::XmlRpcValue params) = 0;

    protected:
      ObserverPluginBase(std::string type);

      /// subscriber functions in order to use the callback queue in the right way
      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj,
                           const ros::TransportHints &transport_hints = ros::TransportHints()
      )
      {
        ros::SubscribeOptions ops;
        ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      /// and the const version
      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj,
                           const ros::TransportHints &transport_hints = ros::TransportHints()
      )
      {
        ros::SubscribeOptions ops;
        ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size,
                           void(T::*fp)(const boost::shared_ptr<M const> &), T *obj,
                           const ros::TransportHints &transport_hints = ros::TransportHints()
      )
      {
        ros::SubscribeOptions ops;
        ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size,
                           void(T::*fp)(const boost::shared_ptr<M const> &) const, T *obj,
                           const ros::TransportHints &transport_hints = ros::TransportHints()
      )
      {
        ros::SubscribeOptions ops;
        ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M),
                           const boost::shared_ptr<T> &obj, const ros::TransportHints &transport_hints = ros::TransportHints())
      {
        ros::SubscribeOptions ops;
        ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.tracked_object = obj;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const,
                           const boost::shared_ptr<T> &obj, const ros::TransportHints &transport_hints = ros::TransportHints())
      {
        ros::SubscribeOptions ops;
        ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.tracked_object = obj;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size,
                           void(T::*fp)(const boost::shared_ptr<M const> &),
                           const boost::shared_ptr<T> &obj, const ros::TransportHints &transport_hints = ros::TransportHints())
      {
        ros::SubscribeOptions ops;
        ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.tracked_object = obj;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      template<class M, class T>
      ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size,
                           void(T::*fp)(const boost::shared_ptr<M const> &) const,
                           const boost::shared_ptr<T> &obj, const ros::TransportHints &transport_hints = ros::TransportHints())
      {
        ros::SubscribeOptions ops;
        ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.transport_hints = transport_hints;
        ops.tracked_object = obj;
        ops.callback_queue = &internal_call_back_queue_;
        return nh_.subscribe(ops);
      }

      void reportError(std::string resource, std::string error_msg, std::string verbose_error_msg, uint32_t error_code);

      void reportStates(std::string resource, std::vector<std::string> states);
    };
}

#endif //TUG_OBSERVERS_CPP_OBSERVERPLUGINBASE_H
