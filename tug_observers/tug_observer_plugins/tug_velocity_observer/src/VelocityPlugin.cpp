//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>

namespace tug_observer_plugins_cpp
{

    VelocityPlugin::VelocityPlugin() : ObserverPluginBase("velocity")
    { }

    void VelocityPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG("VelocityPlugin::initialize 1");
      init(params, this);

      ROS_DEBUG("VelocityPlugin::initialize 2");
      double rate = ProcessYaml::getValue<double>("loop_rate", params, 1.0);
      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1./rate * 1000. * 1000.), boost::bind(&VelocityPlugin::run, this));
      ROS_DEBUG("VelocityPlugin::initialize 3");
    }

    void VelocityPlugin::run()
    {
      ROS_DEBUG("VelocityPlugin::run 1");
      std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > observations = velocityObservations();

      ROS_DEBUG("VelocityPlugin::run 2");
      for(size_t i = 0; i < observations.size(); ++i)
      {
        ROS_DEBUG("VelocityPlugin::run 2.1");
        std::string name = observations[i].head;
        std::vector<Observation> states = observations[i].tail.head;
        ros::Time observation_time = observations[i].tail.tail.head;
        ROS_DEBUG("VelocityPlugin::run 2.2");
        if (states.empty())
        {
          reportError(name, "no_state",
                      "For the input pair with the name '" + name + "' no state could be estimated",
                      tug_observers_msgs::observation::NO_STATE_FITS, observation_time);
        }
        else
        {
          reportStates(name, states, observation_time);
        }
      }

      ROS_DEBUG("VelocityPlugin::run 3");
      flush();
    }
}

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::VelocityPlugin, tug_observers::ObserverPluginBase)
