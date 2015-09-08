//
// Created by clemens on 07.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECK_H
#define TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECK_H

#include <tug_observer_plugin_utils/filter/FilteState.h>

template <class T>
class SingleValueHypothesisCheck
{
public:
    virtual bool checkHypothesis(const FilteState<T>& state) = 0;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECK_H
