//
// Created by clemens on 24.09.15.
//

#ifndef TUG_OBSERVERS_SUBSCRIBERFACADE_H
#define TUG_OBSERVERS_SUBSCRIBERFACADE_H

#include <ros/subscriber.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <ros/ros.h>

class SubscriberFacade
{
protected:
    ros::NodeHandle nh_;

public:
    /// subscriber functions in order to use the callback queue in the right way
    template<class M, class T>
    ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj,
                                      const ros::TransportHints &transport_hints = ros::TransportHints()
    )
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
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
      return nh_.subscribe(ops);
    }
};


#endif //TUG_OBSERVERS_SUBSCRIBERFACADE_H
