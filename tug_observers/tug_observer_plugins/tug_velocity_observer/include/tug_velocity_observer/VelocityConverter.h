//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H

#include <tug_velocity_observer/MovementReading.h>
#include <boost/function.hpp>

class VelocityConverter
{
    boost::function<void (MovementReading)> call_back_;
protected:
    VelocityConverter(boost::function<void (MovementReading)> call_back);
    void sendMovement(MovementReading twist);
public:
    virtual std::string getName() = 0;
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H
