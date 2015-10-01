//
// Created by clemens on 24.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYOBSERVER_H
#define TUG_VELOCITY_OBSERVER_VELOCITYOBSERVER_H

#include <geometry_msgs/TwistStamped.h>
#include <list>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <tug_velocity_observer/VelocityState.h>
#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_velocity_observer/MovementReading.h>
#include <tug_observers/SubscriberFacade.h>
#include <tug_observers/Observation.h>

class VelocityObserver
{
    boost::shared_ptr<VelocityConverter> a_input_;
    boost::shared_ptr<VelocityConverter> b_input_;

    std::list<MovementReading> a_twists_;
    std::list<MovementReading> b_twists_;

    boost::shared_ptr<Filter<double> > diff_x_filter_;
    boost::shared_ptr<Filter<double> > diff_y_filter_;
    boost::shared_ptr<Filter<double> > diff_z_filter_;

    bool use_roll_;
    bool use_pitch_;
    bool use_yaw_;
    boost::shared_ptr<Filter<double> > diff_rot_x_filter_;
    boost::shared_ptr<Filter<double> > diff_rot_y_filter_;
    boost::shared_ptr<Filter<double> > diff_rot_z_filter_;

    std::vector<VelocityState> states_;

    ros::Time current_filter_time_;

    boost::mutex filter_mutex_;

    MovementReading getCompensatedTwist(MovementReading value);

    void updateFilters(MovementReading a, MovementReading b);

    ros::NodeHandle nh_;
    ros::Publisher a_publisher_;
    ros::Publisher b_publisher_;
    ros::Publisher a_paired_publisher_;
    ros::Publisher b_paired_publisher_;
    ros::Publisher filter_result_publisher_;
    ros::AsyncSpinner spinner_;

public:
    VelocityObserver(XmlRpc::XmlRpcValue params, SubscriberFacade* plugin_base);

    void addTwistA(MovementReading value);

    void addTwistB(MovementReading value);

    std::pair<bool, std::vector<Observation> > estimateStates();

    ros::Time getCurrentFilterTime();

    std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYOBSERVER_H
