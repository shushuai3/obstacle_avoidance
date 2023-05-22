#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "flyappy_autonomy_code/flyappy.hpp"
#include <vector>

class FlyappyRos
{
  public:
    FlyappyRos(ros::NodeHandle& nh);

  private:
    void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void gameEndedCallback(const std_msgs::Bool::ConstPtr& msg);

    ros::Publisher pub_acc_cmd_;      ///< Publisher for acceleration command
    ros::Subscriber sub_vel_;         ///< Subscriber for velocity
    ros::Subscriber sub_laser_scan_;  ///< Subscriber for laser scan
    ros::Subscriber sub_game_ended_;  ///< Subscriber for crash detection

    // Mapping parameters
    geometry_msgs::Vector3 pos_drone;
    float dt = 1.0 / 30.0;
    float angle_increment, angle_min;
    float y_max_bound, y_min_bound;
    const int laser_num = 9;
    std::vector<geometry_msgs::Vector3> obs_point_cloud;
    bool init_flag = false;
    float x_min_obs = -10, y_min_safe = -10, y_max_safe = 10;

    // Control parameters
    const float kp_vel_x = 20, kp_vel_y = 25, kp_pos_y = 9.5;
    float desired_vx = 3.35, desired_vy = 0, desired_y = 0;
    bool go_to_opposite_y_flag = false;

    Flyappy flyappy_;  ///< ROS-free main code
};
