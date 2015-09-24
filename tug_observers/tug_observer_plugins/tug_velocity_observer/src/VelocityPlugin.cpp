//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/resource_error.h>

namespace tug_observer_plugins_cpp
{

    VelocityPlugin::VelocityPlugin() : ObserverPluginBase("velocity")
    { }

    void VelocityPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      init(params, this);

      background_thread_ = boost::thread(boost::bind(&VelocityPlugin::run, this));
    }

    void VelocityPlugin::run()
    {
      while(ros::ok())
      {
        std::vector<boost::tuple<std::string, std::vector<std::string>, ros::Time> > observations = velocityObservations();

        for(size_t i = 0; i < observations.size(); ++i)
        {
          ROS_DEBUG_STREAM("VelocityPlugin::run 3.1");
          std::string name = observations[i].head;
          std::vector<std::string> states = observations[i].tail.head;
          ros::Time observation_time = observations[i].tail.tail.head;
          if (states.empty())
          {
            reportError(name, "no_state_" + name,
                        "For the node with the name '" + name + "' no state could be estimated",
                        tug_observers_msgs::resource_error::NO_STATE_FITS, observation_time);
          }
          else
          {
            reportStates(name, states, observation_time);
          }
        }

        //background_rate_.sleep();

        sleep(1);
      }
    }
}

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::VelocityPlugin, tug_observers::ObserverPluginBase)
