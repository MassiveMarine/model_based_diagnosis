//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H
#define TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H

#include <tug_observers/ObserverPluginBase.h>
#include <tug_score_observer/ScoreBase.h>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <tug_time/Timer.h>

namespace tug_observer_plugins_cpp
{
    class ScoresPlugin : public tug_observers::ObserverPluginBase
    {
    private:
      std::vector<boost::shared_ptr<ScoreBase> > bases_;
        boost::shared_ptr<Timer> timer_;

      public:
        ScoresPlugin();
        virtual void initialize(XmlRpc::XmlRpcValue params);
        void run();
    };
}

#endif //TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H
