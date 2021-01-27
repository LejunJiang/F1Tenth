#include <lejun_jiang_roslab/scan_range.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_1 = n_.advertise<std_msgs::Float64>("/closest_point", 1000);
        pub_2 = n_.advertise<std_msgs::Float64>("/farthest_point", 1000);
        pub_3 = n_.advertise<lejun_jiang_roslab::scan_range>("/scan_range", 1000);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {
        std_msgs::Float32 closest_point;
        closest_point.data = lidar_info.ranges[0];
        std_msgs::Float32 farthest_point;
        farthest_point.data = lidar_info.ranges[0];
        for (unsigned int i = 1; i < lidar_info.ranges.size(); i++) {
            if (!std::isinf(lidar_info.ranges[i]) && !std::isnan(lidar_info.ranges[i])) {
                if (lidar_info.ranges[i] < closest_point.data) closest_point.data = lidar_info.ranges[i];
                if (lidar_info.ranges[i] > farthest_point.data) farthest_point.data = lidar_info.ranges[i];
            }
        }
        std_msgs::Float64 closest_point_publish;
        closest_point_publish.data = closest_point.data;
        std_msgs::Float64 farthest_point_publish;
        farthest_point_publish.data = farthest_point.data;
        lejun_jiang_roslab::scan_range scan_range_publish;
        scan_range_publish.closest_point = closest_point.data;
        scan_range_publish.farthest_point = farthest_point.data;
        pub_1.publish(closest_point_publish);
        pub_2.publish(farthest_point_publish);
        pub_3.publish(scan_range_publish);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Publisher pub_3;
    ros::Subscriber sub_;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {
    //Initiate ROS
    ros::init(argc, argv, "lidar_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}