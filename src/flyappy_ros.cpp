#include "flyappy_autonomy_code/flyappy_ros.hpp"
constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;

    /* Goal position calculation & cascaded P control */
    // Update drone global position
    pos_drone.x += msg->x * dt;
    pos_drone.y += msg->y * dt;

    // Remove obstacle points behind the drone
    std::vector<geometry_msgs::Vector3>::iterator it;
    it = remove_if(obs_point_cloud.begin(), obs_point_cloud.end(), [&](geometry_msgs::Vector3 x){ return x.x < pos_drone.x; });
    obs_point_cloud.erase(it, obs_point_cloud.end());

    // Calculate goal position before the drone entering the obstacle area or after it passes the area
    if (pos_drone.x > x_min_obs + 0.45 || pos_drone.x < x_min_obs - 0.2)
    {
        // Calculate when there are more than one obstacle points
        if (obs_point_cloud.size() > 1)
        {
            /* Get the y_min and y_max of the safe area (no obstacles) */
            // Ascending the obstacle point cloud based on y value of each point
            std::sort(obs_point_cloud.begin(), obs_point_cloud.end(), [&](geometry_msgs::Vector3 a, geometry_msgs::Vector3 b){ return a.y < b.y; });

            // Find two neighboring obstacle points that has the maximum y distance
            float max_deltaY = 0;
            for (int i = 0; i < obs_point_cloud.size() - 1; i++)
            {
                if (obs_point_cloud[i + 1].y - obs_point_cloud[i].y > max_deltaY)
                {
                    max_deltaY = obs_point_cloud[i + 1].y - obs_point_cloud[i].y;
                    y_min_safe = obs_point_cloud[i].y;
                    y_max_safe = obs_point_cloud[i + 1].y;
                }
            }

            /* Get the x_min of the obstacle area */
            std::sort(obs_point_cloud.begin(), obs_point_cloud.end(), [&](geometry_msgs::Vector3 a, geometry_msgs::Vector3 b){ return a.x < b.x; });
            x_min_obs = obs_point_cloud[0].x;

            // Debug print
            // ROS_INFO("goal_x: %2.3f, goal_y_min: %2.3f, goal_y_max: %2.3f, drone_x: %2.3f, drone_y: %2.3f", x_min_obs, y_min_safe, y_max_safe, pos_drone.x, pos_drone.y);
        }
    }

    // Check if the safe area is long enough
    if (y_max_safe - y_min_safe < 0.35)
    {
        // Go to the opposition y position of the current drone y position
        if (!go_to_opposite_y_flag)
        {
            desired_y = y_min_bound + y_max_bound - pos_drone.y;
        }
        go_to_opposite_y_flag = true;
        ROS_INFO("safe length: %2.3f, goal_y: %2.3f", y_max_safe - y_min_safe, desired_y);
    }
    else
    {
        // Go to the center y position of the safe area
        desired_y = (y_min_safe + y_max_safe) / 2.0;
        go_to_opposite_y_flag = false;
    }

    /* Cascaded position and velocity control */
    desired_vy = kp_pos_y * (desired_y - pos_drone.y);
    float desired_ax = kp_vel_x * (desired_vx - msg->x);
    float desired_ay = kp_vel_y * (desired_vy - msg->y);
    // Limit the acceleration value
    if (desired_ax > 3) { desired_ax = 3;} else if (desired_ax < -3) {desired_ax = -3;}
    if (desired_ay > 35) { desired_ay = 35;} else if (desired_ay < -35) {desired_ay = -35;}

    acc_cmd.x = desired_ax;
    acc_cmd.y = desired_ay;
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Example of printing laser angle and range
    // ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);

    // Get only once the y-axis bounding and the laser angle constants
    if (!init_flag)
    {
        angle_increment = msg->angle_increment;
        angle_min = msg->angle_min;
        y_min_bound = msg->ranges[0] * sin(-angle_increment * 4) + 0.05;
        y_max_bound = msg->ranges[8] * sin( angle_increment * 4) - 0.05;
        init_flag = true;
    }

    // Update the obstacle point cloud
    for (int i = 0; i < laser_num; i++)
    {
        float angle = angle_min + i * angle_increment;
        float x_laser_end = pos_drone.x + msg->ranges[i] * cos(angle);
        float y_laser_end = pos_drone.y + msg->ranges[i] * sin(angle);

        // Add valid obstacle points to the obstacle point cloud
        if (y_laser_end > y_min_bound && y_laser_end < y_max_bound && msg->ranges[i] < 3.5)
        {
            geometry_msgs::Vector3 tmp_point;
            tmp_point.x = x_laser_end;
            tmp_point.y = y_laser_end;

            // Save the point that is not close to any existed points (>0.02m here)
            float distance_min = 10;
            for (auto point: obs_point_cloud)
            {
                float distance = sqrt(pow((tmp_point.x - point.x), 2) + pow((tmp_point.y - point.y), 2));
                if (distance < distance_min)
                {
                    distance_min = distance;
                } 
            }
            // Also neglect the points far away from the drone (2.1m here)
            if (distance_min > 0.02 && (tmp_point.x - pos_drone.x < 2.1))
            {
                obs_point_cloud.push_back(tmp_point);
            }
        }
    }
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_ = {};

    // Reinitialize the parameters
    pos_drone.x = 0;
    pos_drone.y = 0;
    x_min_obs = -10;
    y_min_safe = -10;
    y_max_safe = 10;
    desired_vy = 0;
    go_to_opposite_y_flag = false;
    desired_y = 0;
    obs_point_cloud.clear();
}
