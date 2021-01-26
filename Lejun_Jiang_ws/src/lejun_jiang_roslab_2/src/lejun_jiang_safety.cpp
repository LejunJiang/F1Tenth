#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "math.h"

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_1 = n_.advertise<std_msgs::Bool>("/brake_bool", 1000);
        pub_2 = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);

        //Topic you want to subscribe
        sub_1 = n_.subscribe("/scan", 1000, &SubscribeAndPublish::callback_laserscan, this);
        sub_2 = n_.subscribe("/odom", 1000, &SubscribeAndPublish::callback_odometry, this);
    }

    void callback_laserscan(const sensor_msgs::LaserScan& lidar_info) {
        lidar_info_local = lidar_info;
    }

    void callback_odometry(const nav_msgs::Odometry& odometry_info) {
        odometry_info_local = odometry_info;
    }

    void publish() {
        double TTC_threshold = 0.4;
        double min_TTC = 100;
        double v_x = odometry_info_local.twist.twist.linear.x;
        double v_y = odometry_info_local.twist.twist.linear.y;
        for (unsigned int i = 0; i < lidar_info_local.ranges.size(); i++) {
            if (!std::isinf(lidar_info_local.ranges[i]) || !std::isnan(lidar_info_local.ranges[i])) {
                double distance = lidar_info_local.ranges[i];
                double angle = lidar_info_local.angle_min + lidar_info_local.angle_increment * i;
                double distance_derivative = cos(angle) * v_x + sin(angle) * v_y;
                if (distance_derivative > 0 && distance / distance_derivative < min_TTC) min_TTC = distance / distance_derivative;
            }
        }
        if (min_TTC <= TTC_threshold) {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = true;
            pub_1.publish(brake_bool_result);
            ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
            ackermann_drive_result.drive.speed = 0.0;
            pub_2.publish(ackermann_drive_result);
        } else {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = false;
            pub_1.publish(brake_bool_result);
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    sensor_msgs::LaserScan lidar_info_local;
    nav_msgs::Odometry odometry_info_local;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {
    //Initiate ROS
    ros::init(argc, argv, "lejun_jiang_safety");
    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        SAPObject.publish();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}