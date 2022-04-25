#include "ros/ros.h"
#include "std_msgs/String.h"
#include "math.h"
#include <ceres_msgs/MotorRotations.h>
#include <ceres_msgs/WheelPositions.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher odom_pub;

void motorRotationCallback(const ceres_msgs::WheelPositions msg)
{
    ROS_INFO("Motor Rotations: %f", msg.pos_front_left);
    

    static float prev_pos_front_left = 0;
    static float prev_pos_front_right = 0;
    static float prev_pos_rear_left = 0;
    static float prev_pos_rear_right = 0;
    static ros::Time prev_time = ros::Time::now();

    static bool first_message = true;
    static float x = 0;
    static float y = 0;
    static float theta = 0;
    static int seq = 0;

    if (first_message){
        prev_pos_front_left = msg.pos_front_left;
        prev_pos_front_right = msg.pos_front_right;
        prev_pos_rear_left = msg.pos_rear_left;
        prev_pos_rear_right = msg.pos_rear_right;
        prev_time = msg.stamp;
        first_message = false;
    }
    else{
        nav_msgs::Odometry odom;
        float dt = (msg.stamp - prev_time).toSec();
        float d_front_left = msg.pos_front_left - prev_pos_front_left;
        float d_front_right = msg.pos_front_right - prev_pos_front_right;
        float d_rear_left = msg.pos_rear_left - prev_pos_rear_left;
        float d_rear_right = msg.pos_rear_right - prev_pos_rear_right;

        float d_left = (d_front_left + d_rear_left) / 2;
        float d_right = (d_front_right + d_rear_right) / 2;

        float d_s = (d_left + d_right) / 2;
        float d_theta = (d_right - d_left) / 0.435;

        x += d_s * cos(theta + d_theta / 2);
        y += d_s * sin(theta + d_theta / 2);
        theta += d_theta;

        prev_pos_front_left = msg.pos_front_left;
        prev_pos_front_right = msg.pos_front_right;
        prev_pos_rear_left = msg.pos_rear_left;
        prev_pos_rear_right = msg.pos_rear_right;

        prev_time = msg.stamp;

        odom.header.stamp = msg.stamp;
        odom.header.seq = seq++;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;

        tf2::Quaternion rot;
        rot.setRPY(0, 0, theta);
        odom.pose.pose.orientation = tf2::toMsg(rot);
        odom.twist.twist.linear.x = d_s / dt;
        odom.twist.twist.angular.z = d_theta / dt;
        odom_pub.publish(odom); 
    }

   

    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_estimator");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom_est", 1000);
  ros::Subscriber sub = n.subscribe("/wheel_positions", 1000, motorRotationCallback);
  ros::spin();

  return 0;
}

