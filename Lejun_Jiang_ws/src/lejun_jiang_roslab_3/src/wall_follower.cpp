#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "math.h"

#define KP 1.00
#define KD 0.001
#define KI 0.005
#define SERVO_OFFSET 0.00
#define ANGLE_RANGE 270
#define DISIRED_DISTANCE_RIGHT 0.9
#define DISIRED_DISTANCE_LEFT 1.20
#define CAR_LENGTH 0.50
#define PI 3.1415927

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {
        unsigned int b_indx = (unsigned int)(floor((90.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        double b_angle = 90.0 / 180.0 * PI;
        double a_angle = 45.0 / 180.0 * PI;
        unsigned int a_indx;
        if (lidar_info.angle_min > 45.0 / 180.0 * PI) {
            a_angle = lidar_info.angle_min;
            a_indx = 0;
        } else {
            a_indx = (unsigned int)(floor((45.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        }
        // unsigned int c_indx = (unsigned int)(floor((70.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        // double c_angle = 70.0 / 180.0 * PI;
        double a_range = 0.0;
        double b_range = 0.0;
        // double c_range = 0.0;
        if (!std::isinf(lidar_info.ranges[a_indx]) && !std::isnan(lidar_info.ranges[a_indx])) {
            a_range = lidar_info.ranges[a_indx];
        } else {
            a_range = 100.0;
        }
        if (!std::isinf(lidar_info.ranges[b_indx]) && !std::isnan(lidar_info.ranges[b_indx])) {
            b_range = lidar_info.ranges[b_indx];
        } else {
            b_range = 100.0;
        }
        // if (!std::isinf(lidar_info.ranges[c_indx]) && !std::isnan(lidar_info.ranges[c_indx])) {
        //     c_range = lidar_info.ranges[c_indx];
        // }
        ROS_INFO_STREAM("d_t1");
        // ROS_INFO_STREAM(lidar_info.angle_min);
        // ROS_INFO_STREAM(lidar_info.angle_max);
        // ROS_INFO_STREAM(lidar_info.angle_increment);
        // ROS_INFO_STREAM(a_angle);
        // ROS_INFO_STREAM(b_angle);
        // ROS_INFO_STREAM(a_indx);
        // ROS_INFO_STREAM(b_indx);
        // ROS_INFO_STREAM(a_range);
        // ROS_INFO_STREAM(b_range);
        double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle)));
        // double alpha_2 = atan((c_range * cos(b_angle - c_angle) - b_range) / (c_range * sin(b_angle - c_angle)));
        // alpha = (alpha + alpha_2) / 2.0;
        double d_t = b_range * cos(alpha);
        // double d_t1 = d_t + velocity * del_time * sin(alpha);
        double d_t1 = d_t + 1.00 * sin(alpha);
        error = DISIRED_DISTANCE_LEFT - d_t1;
        ROS_INFO_STREAM(d_t1);
        ROS_INFO_STREAM(error);
        ROS_INFO_STREAM(del_time);
        SubscribeAndPublish::pid_control();
    }

    void pid_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        double tmoment = ros::Time::now().toSec();
        del_time = tmoment - prev_tmoment;
        integral += prev_error * del_time;
        ackermann_drive_result.drive.steering_angle = -(KP * error + KD * (error - prev_error) / del_time + KI * integral);
        prev_tmoment = tmoment;
        if (abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 0.5;
            velocity = 0.5;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 1.0;
            velocity = 0.5;
        } else {
            ackermann_drive_result.drive.speed = 1.5;
            velocity = 0.5;
        }
        pub_.publish(ackermann_drive_result);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double prev_error = 0.0;
    double prev_tmoment = ros::Time::now().toSec();
    double error = 0.0;
    double integral = 0.0;
    double velocity = 0.0;
    double del_time = 0.0;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {
    //Initiate ROS
    ros::init(argc, argv, "wall_follower");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}