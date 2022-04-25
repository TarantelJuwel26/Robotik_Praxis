#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <ceres_msgs/MotorRotations.h>
#include <ceres_msgs/WheelPositions.h>

ros::Publisher wheel_pub;

float rot_to_dis(float rot)
{
  return rot * 0.13;
}
void motorRotationCallback(const ceres_msgs::MotorRotations msg)
{
    ROS_INFO("Motor Rotations: %f", msg.rot_front_left);
    ceres_msgs::WheelPositions wheel_positions;

    wheel_positions.stamp = msg.stamp;
    wheel_positions.pos_front_left = rot_to_dis(msg.rot_front_left);
    wheel_positions.pos_front_right = rot_to_dis(msg.rot_front_right);
    wheel_positions.pos_rear_left = rot_to_dis(msg.rot_rear_left);
    wheel_positions.pos_rear_right = rot_to_dis(msg.rot_rear_right);

    if(wheel_pub && wheel_pub.getNumSubscribers() > 0)
    {
        wheel_pub.publish(wheel_positions);
    }
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  wheel_pub = n.advertise<ceres_msgs::WheelPositions>("wheel_positions", 1000);
  ros::Subscriber sub = n.subscribe("/wheel_rotations", 1000, motorRotationCallback);
  ros::spin();

  return 0;
}

