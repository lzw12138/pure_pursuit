#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "path");
    ros::NodeHandle nh;
    ros::Publisher path_pub =nh.advertise<nav_msgs::Path>("global_plan",10, true);
    std::ifstream infile;
    infile.open("/home/lzw/tianbot_ws/src/f1tenth_simulator/data/path.txt");
    assert(infile.is_open());  //若失败,则输出错误消息,并终止程序运行
    std::vector<std::pair<double, double>> xy_points;
    geometry_msgs::PoseStamped path_element;
    nav_msgs::Path path;
    std::string s;
    std::string x;
    std::string y;
    while (getline(infile, s)) {
        std::stringstream word(s);
        word >> x;
        word >> y;
        double pt_x = std::atof(x.c_str());
        double pt_y = std::atof(y.c_str());
        xy_points.push_back(std::make_pair(pt_x, pt_y));
        path_element.pose.position.x = pt_x;
        path_element.pose.position.y = pt_y;
        path_element.pose.orientation.x = 0;
        path_element.pose.orientation.y = 0;
        path_element.pose.orientation.z = 0;
        path_element.pose.orientation.w = 1;
        path_element.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        path.poses.push_back(path_element);
    }
    infile.close();
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        path_pub.publish(path);
        loop_rate.sleep();
    }



}