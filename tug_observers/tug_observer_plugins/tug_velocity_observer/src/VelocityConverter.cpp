//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverter.h>

VelocityConverter::VelocityConverter(boost::function<void (MovementReading)> call_back) : call_back_(call_back)
{ }

void VelocityConverter::sendMovement(MovementReading twist)
{
  call_back_(twist);
}
