#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "target_size");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float32>("target_size", 1);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {

    std_msgs::Float32 msg;

    msg.data = 4000;

    ROS_INFO("%f", msg.data);

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

