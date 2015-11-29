/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_OBSERVERS_SUBSCRIBERFACADE_H
#define TUG_OBSERVERS_SUBSCRIBERFACADE_H

#include <ros/subscriber.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <ros/ros.h>
#include <string>

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
                              const boost::shared_ptr<T> &obj,
                              const ros::TransportHints &transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      ops.tracked_object = obj;
      return nh_.subscribe(ops);
    }

    template<class M, class T>
    ros::Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const,
                              const boost::shared_ptr<T> &obj,
                              const ros::TransportHints &transport_hints = ros::TransportHints())
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
                              const boost::shared_ptr<T> &obj,
                              const ros::TransportHints &transport_hints = ros::TransportHints())
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
                              const boost::shared_ptr<T> &obj,
                              const ros::TransportHints &transport_hints = ros::TransportHints())
    {
      ros::SubscribeOptions ops;
      ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
      ops.transport_hints = transport_hints;
      ops.tracked_object = obj;
      return nh_.subscribe(ops);
    }
};


#endif  // TUG_OBSERVERS_SUBSCRIBERFACADE_H
