#include <ros/ros.h>
#include <diagnosis_msgs/Observations.h>
class Listen
{
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
	public:
	Listen(ros::NodeHandle n)
		{
 				nh_ = n;
 				sub_ =  nh_.subscribe("/observations", 1, &Listen::callback, this);
		}//Constructor
	void callback(const diagnosis_msgs::ObservationsConstPtr& msg)
		{
			ROS_INFO("recieved ");
		}
	void start()
		{
			while(ros::ok())
 					ros::spinOnce();
		}//start
};

int main(int argc, char **argv)
{
ros::init(argc, argv, "Listen_class_node");
ros::NodeHandle n;
Listen *listen=new Listen(n);
listen->start();
return 0;
}

