#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <string>

#include "grid.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "trajectory_planner_cpp");

    ros::NodeHandle nh;

    ros::Rate loopRate(1);

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "carcaca";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "carcaca";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.025;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    TrajPlanner planner = TrajPlanner();
    planner.inflateGrid(15);

    float f = 0.0;

    // int initial_x = 90;
    // int initial_y = 47;

    // int end_x = 75;
    // int end_y = 194;

    int initial_x = 75;
    int initial_y = 194;

    int end_x = 150;
    int end_y = 50;

    // int initial_x = 90;
    // int initial_y = 47;
    // int end_x = 150;
    // int end_y = 210;

    // int end_x = 280;
    // int end_y = 855;
    
    // int initial_x = 100;
    // int initial_y = 94;

    // final
    // int end_x = 158;
    // int end_y = 427;
    // rota curta
    // int end_x = 200;
    // int end_y = 220;

    planner.genTrajectory(cv::Point(initial_x, initial_y), cv::Point(end_x, end_y), 200, 100);

    vector<geometry_msgs::Point> shortestPath = planner.getShortestPath();
    for (int i=0; i<shortestPath.size(); i++) {
        shortestPath[i].x = (shortestPath[i].x * 0.05);
        shortestPath[i].y = (shortestPath[i].y * 0.05);

        line_strip.points.push_back(shortestPath[i]);
    }

    ros::spinOnce();
    loopRate.sleep();

    marker_pub.publish(line_strip);

    std::cout << "enviando rota" << std::endl;
    for(int i=0; i<line_strip.points.size(); i++) {
        std::cout << "pt " << i << "\n" << line_strip.points[i] << "\n";
    }
    std::cout << std::endl;

    planner.show(0);

    return 0;
}
