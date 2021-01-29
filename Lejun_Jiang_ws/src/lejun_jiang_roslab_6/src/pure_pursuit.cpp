#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>

#include "csv.h"
#include "math.h"

#define LOOKAHEAD_DISTANCE 1.20
#define KP 1.00

#define PI 3.1415927

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_1 = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        pub_2 = n_.advertise<visualization_msgs::Marker>("/env_viz", 1000);
        pub_3 = n_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/pf/pose/odom", 1000, &SubscribeAndPublish::callback, this);
        // sub_ = n_.subscribe("/odom", 1000, &SubscribeAndPublish::callback, this);

        // read in all the data
        io::CSVReader<3> in("data.csv");
        double x;
        double y;
        double heading;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;  // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        geometry_msgs::Point points;
        while (in.read_row(x, y, heading)) {
            xes.push_back(x);
            yes.push_back(y);
            headings.push_back(heading);
            points.x = x;
            points.y = y;
            points.z = 0.0;
            marker.points.push_back(points);
            // markerArray.markers.push_back(marker);
        }
        // mar_id++;
    }

    void callback(const nav_msgs::Odometry& odometry_info) {
        x_current = odometry_info.pose.pose.position.x;
        y_current = odometry_info.pose.pose.position.y;
        double siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z);
        heading_current = std::atan2(siny_cosp, cosy_cosp);
        if (!flag) {
            double shortest_distance = 100.0;
            for (unsigned int i = 0; i < xes.size(); i++) {
                if ((xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current) < shortest_distance) {
                    shortest_distance = (xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current);
                    current_indx = i;
                }
            }
            flag = true;
        }
        while (std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current)) < LOOKAHEAD_DISTANCE) {
            current_indx++;
            if (current_indx > xes.size() - 1) {
                current_indx = 0;
            }
        }
        double real_distance = std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current));
        double lookahead_angle = std::atan2(yes[current_indx] - y_current, xes[current_indx] - x_current);
        double del_y = real_distance * std::sin(lookahead_angle - heading_current);
        angle = KP * 2.00 * del_y / (real_distance * real_distance);
        geometry_msgs::Point points;
        visualization_msgs::Marker marker_2;
        points.x = xes[current_indx];
        points.y = yes[current_indx];
        points.z = 0.0;
        marker_2.points.push_back(points);
        marker_2.header.frame_id = "map";
        marker_2.header.stamp = ros::Time();
        marker_2.id = 0;
        marker_2.type = visualization_msgs::Marker::POINTS;
        marker_2.action = visualization_msgs::Marker::ADD;
        marker_2.pose.position.x = 0.0;
        marker_2.pose.position.y = 0.0;
        marker_2.pose.position.z = 0.0;
        marker_2.pose.orientation.x = 0.0;
        marker_2.pose.orientation.y = 0.0;
        marker_2.pose.orientation.z = 0.0;
        marker_2.pose.orientation.w = 1.0;
        marker_2.scale.x = 0.2;
        marker_2.scale.y = 0.2;
        marker_2.color.a = 1.0;  // Don't forget to set the alpha!
        marker_2.color.r = 1.0;
        marker_2.color.g = 0.0;
        marker_2.color.b = 0.0;
        SubscribeAndPublish::reactive_control();
        pub_2.publish(marker);
        pub_3.publish(marker_2);
    }

    void reactive_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = angle;
        if (std::abs(angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 5.0;
        } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 5.0;
        } else {
            ackermann_drive_result.drive.speed = 5.0;
        }
        pub_1.publish(ackermann_drive_result);
        // ROS_INFO_STREAM(angle);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Publisher pub_3;
    ros::Subscriber sub_;
    visualization_msgs::Marker marker;
    double angle;
    double x_current;
    double y_current;
    double heading_current;
    unsigned int current_indx;
    bool flag = false;
    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {
    //Initiate ROS
    ros::init(argc, argv, "pure_pursuit");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}