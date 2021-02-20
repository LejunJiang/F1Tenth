// RRT and RRT star implementation
// Author: Lejun Jiang, Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Reference: https://arxiv.org/pdf/1105.1186.pdf

#include "lejun_jiang_roslab_7/rrt.h"

// define parameters here
#define STEER_LENGTH 0.30
#define TERMINATE_LENGTH 0.10
#define LOOKAHEAD_DISTANCE 0.60
#define KP 1.00
#define PI 3.1415927
#define ETA 0.60
#define MAX_ITERATION 100

// convert global frame to grid frame
// grid size: 0.05m * 0.05m
std::vector<unsigned int> convert_frame(double x_global, double y_global, double x_off = 14.50, double y_off = 0.70) {
    double x_grid = x_global + x_off;
    double y_grid = y_global + y_off;
    unsigned int x_grid_int = (unsigned int)std::round(x_grid / 0.05);
    unsigned int y_grid_int = (unsigned int)std::round(y_grid / 0.05);
    if (x_grid_int > 100000) {
        x_grid_int = 0;
    }
    if (y_grid_int > 100000) {
        y_grid_int = 0;
    }
    return {x_grid_int, y_grid_int};
}

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh, RRT_type rrt_Type) : nh_(nh), gen((std::random_device())()), rrt_type(rrt_Type) {
    // Skipped: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    // nh_.getParam("pose_topic", pose_topic);
    // nh_.getParam("scan_topic", scan_topic);
    std::string pose_topic, scan_topic;
    pose_topic = "/odom";
    scan_topic = "/scan";

    // ROS publishers
    // create publishers for the the drive topic, and other topics you might need
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
    mapvisual_pub_ = nh_.advertise<visualization_msgs::Marker>("/env_viz", 1000);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);
    waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/static_viz", 1000);
    edges_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_lines", 1000);

    // ROS subscribers
    // create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    // create an occupancy grid
    occupancy_grids_prior = std::vector<std::vector<bool>>(500, std::vector<bool>(200, true));

    // set the occupancy grid according to knowledge about the levine hall
    for (unsigned int i = 0; i <= y_rr; i++) {  // right wall
        for (unsigned int j = 0; j < occupancy_grids_prior.size(); j++) {
            occupancy_grids_prior[j][i] = false;
        }
    }
    for (unsigned int i = y_ll; i < occupancy_grids_prior[0].size(); i++) {  // left wall
        for (unsigned int j = 0; j < occupancy_grids_prior.size(); j++) {
            occupancy_grids_prior[j][i] = false;
        }
    }
    for (unsigned int i = 0; i < occupancy_grids_prior[0].size(); i++) {  // bot wall
        for (unsigned int j = 0; j <= x_bb; j++) {
            occupancy_grids_prior[j][i] = false;
        }
    }
    for (unsigned int i = 0; i < occupancy_grids_prior[0].size(); i++) {  // upper wall
        for (unsigned int j = x_tt; j < occupancy_grids_prior.size(); j++) {
            occupancy_grids_prior[j][i] = false;
        }
    }
    for (unsigned int i = y_rl; i <= y_lr; i++) {  // inner parts
        for (unsigned int j = x_bt; j <= x_tb; j++) {
            occupancy_grids_prior[j][i] = false;
        }
    }

    occupancy_grids = occupancy_grids_prior;
    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan &scan_msg) {
    // The scan callback, update your occupancy grid here
    // each point scanned results in a square of 0.6m * 0.6m blocked around it
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // reset the occupancy grids
    occupancy_grids = occupancy_grids_prior;
    // update occupancy grid
    double rear_to_lidar = 0.29275;
    double x_lidar = x_current + rear_to_lidar * std::cos(heading_current);
    double y_lidar = y_current + rear_to_lidar * std::sin(heading_current);
    for (unsigned int i = 0; i < scan_msg.ranges.size(); i++) {
        if (!std::isinf(scan_msg.ranges[i]) && !std::isnan(scan_msg.ranges[i])) {
            double distance = scan_msg.ranges[i];
            double local_angle = scan_msg.angle_min + scan_msg.angle_increment * i;
            double global_angle = local_angle + heading_current;
            double x_obstacle = x_lidar + distance * std::cos(global_angle);
            double y_obstacle = y_lidar + distance * std::sin(global_angle);
            std::vector<unsigned int> grid_coordinates = convert_frame(x_obstacle, y_obstacle);
            for (unsigned int j = std::max((int)grid_coordinates[0] - 6, 0); j <= std::min((int)grid_coordinates[0] + 6, (int)occupancy_grids.size() - 1); j++) {
                for (unsigned int k = std::max((int)grid_coordinates[1] - 6, 0); k <= std::min((int)grid_coordinates[1] + 6, (int)occupancy_grids[0].size() - 1); k++) {
                    occupancy_grids[j][k] = false;
                }
            }
        }
    }
}

void RRT::pf_callback(const nav_msgs::Odometry &odometry_info) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // fetch the current location, can be done by a particle filter
    x_current = odometry_info.pose.pose.position.x;
    y_current = odometry_info.pose.pose.position.y;

    // set the global goal point depending on the car's location
    if (x_current <= 7.00 && y_current <= 2.34) {  // on the right side of the loop
        x_goal = x_current + 2.30;
        y_goal = -0.145;
        x_limit_top = x_current + 2.50;
        x_limit_bot = x_current;
        y_limit_left = 0.37;
        y_limit_right = -0.66;
    } else if (x_current > 7.00 && y_current <= 6.15) {  // on the top side of the loop
        x_goal = 9.575;
        y_goal = y_current + 2.30;
        x_limit_top = 10.03;
        x_limit_bot = 9.12;
        y_limit_left = y_current + 2.50;
        y_limit_right = y_current;
    } else if (x_current >= -11.26 && y_current > 6.15) {  // on the left side of the loop
        x_goal = x_current - 2.30;
        y_goal = 8.65;
        x_limit_top = x_current;
        x_limit_bot = x_current - 2.50;
        y_limit_left = 9.15;
        y_limit_right = 8.15;
    } else if (x_current < -11.26 && y_current > 2.34) {  // on the bottom side of the loop
        x_goal = -13.79;
        y_goal = y_current - 2.30;
        x_limit_top = -13.32;
        x_limit_bot = -14.26;
        y_limit_left = y_current;
        y_limit_right = y_current - 2.50;
    }

    // tree as std::vector
    std::vector<Node> tree;

    // the RRT main loop

    // define the starter node
    Node start;
    start.x = x_current;
    start.y = y_current;
    start.is_root = true;
    tree.push_back(start);

    // For drawing the sampled points
    marker.points.clear();

    // points to be added for plotting
    geometry_msgs::Point points;

    // vector to store the final path
    std::vector<Node> paths;

    // each loop creates a new sample in the space, generate up to MAX_ITERATION samples due to on-board computation constraints
    for (unsigned int i = 0; i < MAX_ITERATION; i++) {
        std::vector<double> sampled_point = sample();               // sample the free space
        unsigned int nearest_point = nearest(tree, sampled_point);  // get the tree's nearest point
        Node new_node = steer(tree[nearest_point], sampled_point);  // steer the tree toward the sampled point, get new point
        new_node.parent = nearest_point;                            // set the parent of the new point
        if (!check_collision(tree[nearest_point], new_node)) {      // collision checking for connecting the new point to the tree
            // if algorithm RRT* star is chosen, the block in the if statement is performed
            if (rrt_type == RRT_type::RRT_star) {
                std::vector<int> near_set = near(tree, new_node);  // set of points in the neighborhood of the new point
                // find the points in the neighborhood through which minimum cost can be obtained to reach the new point
                double min_cost = cost(tree, tree[nearest_point]) + line_cost(new_node, tree[nearest_point]);
                for (unsigned int j = 0; j < near_set.size(); j++) {
                    if ((!check_collision(tree[near_set[j]], new_node)) && (cost(tree, tree[near_set[j]]) + line_cost(new_node, tree[near_set[j]]) < min_cost)) {
                        new_node.parent = near_set[j];
                        min_cost = cost(tree, tree[near_set[j]]) + line_cost(new_node, tree[near_set[j]]);
                    }
                }
                // rewire the tree to get lower cost for other points in the neighborhood
                new_node.cost = min_cost;
                for (unsigned int j = 0; j < near_set.size(); j++) {
                    if ((!check_collision(tree[near_set[j]], new_node)) && (min_cost + line_cost(new_node, tree[near_set[j]]) < cost(tree, tree[near_set[j]]))) {
                        tree[near_set[j]].parent = tree.size();
                        tree[near_set[j]].cost = min_cost + line_cost(new_node, tree[near_set[j]]);
                    }
                }
            }

            tree.push_back(new_node);  // add the new point to the tree
            // visualize the new point in the rviz
            points.x = new_node.x;
            points.y = new_node.y;
            points.z = 0.0;
            marker.points.push_back(points);
            if (is_goal(new_node, x_goal, y_goal)) {  // check if the goal point is reached
                paths = find_path(tree, new_node);    // return the generated path
                break;
            }
        }
    }
    // find the desired way point
    for (unsigned int i = 0; i < paths.size(); i++) {
        if (std::sqrt(std::pow((paths[paths.size() - 1 - i].x - x_current), 2) + std::pow((paths[paths.size() - 1 - i].y - y_current), 2)) >= LOOKAHEAD_DISTANCE) {
            x_target = paths[paths.size() - 1 - i].x;
            y_target = paths[paths.size() - 1 - i].y;
            // ROS_INFO_STREAM("updated");
            break;
        }
    }

    // compute current heading
    double siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z);
    heading_current = std::atan2(siny_cosp, cosy_cosp);
    // using Pure Pursuit algorithm to navigate the car
    double real_distance = std::sqrt((x_target - x_current) * (x_target - x_current) + (y_target - y_current) * (y_target - y_current));
    double lookahead_angle = std::atan2(y_target - y_current, x_target - x_current);
    double del_y = real_distance * std::sin(lookahead_angle - heading_current);
    angle = KP * 2.00 * del_y / (real_distance * real_distance);
    reactive_control();

    // publish the sampled points to be visualized in rviz
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    mapvisual_pub_.publish(marker);

    // publish the global goal point to be visualized in rviz
    marker_2.points.clear();
    points.x = x_goal;
    points.y = y_goal;
    marker_2.points.push_back(points);
    marker_2.header.frame_id = "map";
    marker_2.header.stamp = ros::Time();
    marker_2.type = visualization_msgs::Marker::POINTS;
    marker_2.action = visualization_msgs::Marker::ADD;
    marker_2.scale.x = 0.2;
    marker_2.scale.y = 0.2;
    marker_2.color.a = 1.0;  // Don't forget to set the alpha!
    marker_2.color.r = 1.0;
    marker_2.color.g = 0.0;
    marker_2.color.b = 0.0;
    points_pub_.publish(marker_2);

    // publish the target way point to be visualized in rviz
    marker_3.points.clear();
    points.x = x_target;
    points.y = y_target;
    marker_3.points.push_back(points);
    marker_3.header.frame_id = "map";
    marker_3.header.stamp = ros::Time();
    marker_3.type = visualization_msgs::Marker::POINTS;
    marker_3.action = visualization_msgs::Marker::ADD;
    marker_3.scale.x = 0.2;
    marker_3.scale.y = 0.2;
    marker_3.color.a = 1.0;  // Don't forget to set the alpha!
    marker_3.color.r = 0.0;
    marker_3.color.g = 1.0;
    marker_3.color.b = 0.0;
    waypoint_pub_.publish(marker_3);

    // publish the paths to be visualized in rviz
    marker_4.header.frame_id = "map";
    marker_4.header.stamp = ros::Time();
    marker_4.type = visualization_msgs::Marker::LINE_STRIP;
    marker_4.action = visualization_msgs::Marker::ADD;
    marker_4.scale.x = 0.01;
    marker_4.scale.y = 0.1;
    marker_4.color.a = 1.0;  // Don't forget to set the alpha!
    marker_4.color.r = 0.0;
    marker_4.color.g = 1.0;
    marker_4.color.b = 0.0;
    edges_pub_.publish(marker_4);
    // path found as Path message
}

void RRT::reactive_control() {
    // This method publishes the desired steering angle command to the appropriate topic
    ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
    ackermann_drive_result.drive.steering_angle = angle;
    if (std::abs(angle) > 20.0 / 180.0 * PI) {
        ackermann_drive_result.drive.speed = 1.0;  // 5.0
    } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
        ackermann_drive_result.drive.speed = 1.0;
    } else {
        ackermann_drive_result.drive.speed = 1.5;
    }
    drive_pub_.publish(ackermann_drive_result);
    // ROS_INFO_STREAM(angle);
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // global coordinates of the car
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    x_dist = std::uniform_real_distribution<>(x_limit_bot, x_limit_top);
    y_dist = std::uniform_real_distribution<>(y_limit_right, y_limit_left);
    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));
    return sampled_point;
}

unsigned int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (unsigned int): index of nearest node on the tree

    int nearest_node = 0;
    double min_distance = std::pow((tree[0].x - sampled_point[0]), 2) + std::pow((tree[0].y - sampled_point[1]), 2);
    for (unsigned int ite = 1; ite < tree.size(); ite++) {
        if (std::pow((tree[ite].x - sampled_point[0]), 2) + std::pow((tree[ite].y - sampled_point[1]), 2) < min_distance) {
            min_distance = std::pow((tree[ite].x - sampled_point[0]), 2) + std::pow((tree[ite].y - sampled_point[1]), 2);
            nearest_node = ite;
        }
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    double act_distance = std::sqrt(std::pow((nearest_node.x - sampled_point[0]), 2) + std::pow((nearest_node.y - sampled_point[1]), 2));
    new_node.x = nearest_node.x + STEER_LENGTH / act_distance * (sampled_point[0] - nearest_node.x);
    new_node.y = nearest_node.y + STEER_LENGTH / act_distance * (sampled_point[1] - nearest_node.y);
    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    for (unsigned int i = 0; i <= 100; i++) {
        std::vector<unsigned int> coordinate = convert_frame(nearest_node.x + i * 0.01 * (new_node.x - nearest_node.x), nearest_node.y + i * 0.01 * (new_node.y - nearest_node.y));
        if (occupancy_grids[coordinate[0]][coordinate[1]] == false) {
            collision = true;
        }
    }
    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    if (std::sqrt(std::pow((latest_added_node.x - goal_x), 2) + std::pow((latest_added_node.y - goal_y), 2)) <= TERMINATE_LENGTH) {
        close_enough = true;
    }
    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;
    geometry_msgs::Point points;
    marker_4.points.clear();
    points.x = x_goal;
    points.y = y_goal;
    points.z = 0.0;
    marker_4.points.push_back(points);
    points.x = latest_added_node.x;
    points.y = latest_added_node.y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    found_path.push_back(latest_added_node);
    Node next_node = tree[latest_added_node.parent];
    while (!next_node.is_root) {
        found_path.push_back(next_node);
        next_node = tree[next_node.parent];
        points.x = next_node.x;
        points.y = next_node.y;
        points.z = 0.0;
        marker_4.points.push_back(points);
    }
    found_path.push_back(tree[0]);
    points.x = tree[0].x;
    points.y = tree[0].y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    cost = node.cost;
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    cost = std::sqrt(std::pow((n1.x - n2.x), 2) + std::pow((n1.y - n2.y), 2));
    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    for (unsigned int ite = 0; ite < tree.size(); ite++) {
        if (std::sqrt(std::pow((tree[ite].x - node.x), 2) + std::pow((tree[ite].y - node.y), 2)) < ETA) {
            neighborhood.push_back(ite);
        }
    }
    return neighborhood;
}