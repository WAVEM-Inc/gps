#ifndef VEL_BYPASS_LISTENER_HPP__
#define VEL_BYPASS_LISTENER_HPP__
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <unistd.h>
using GpsMSG = sensor_msgs::msg::NavSatFix ;
class GpsLogger : public rclcpp::Node{
	int fd_ori_log;
	int fd_ori_gpx;
	int fd_filter_log;
	int fd_filter_gpx;
	time_t now;
	struct tm *fix_t;
    private :
        rclcpp::Subscription<GpsMSG>::SharedPtr sub_ori_gps_;
        rclcpp::Subscription<GpsMSG>::SharedPtr sub_filter_gps_;
        void gps_ori_callback(const std::shared_ptr<GpsMSG> fix);
        void gps_filter_callback(const std::shared_ptr<GpsMSG> fix);
    public :
        GpsLogger();
	

};
#endif
