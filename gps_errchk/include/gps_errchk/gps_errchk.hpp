#ifndef VEL_BYPASS_LISTENER_HPP__
#define VEL_BYPASS_LISTENER_HPP__
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <unistd.h>
using GpsMSG = sensor_msgs::msg::NavSatFix ;
using OdomMSG = nav_msgs::msg::Odometry;
using ImuMSG = sensor_msgs::msg::Imu;
class GpsErrchk : public rclcpp::Node{
	private :
		rclcpp::Subscription<GpsMSG>::SharedPtr sub_ori_gps_;
		rclcpp::Subscription<GpsMSG>::SharedPtr sub_filter_gps_;

		rclcpp::Subscription<OdomMSG>::SharedPtr sub_odom_global_;
		rclcpp::Subscription<OdomMSG>::SharedPtr sub_odom_gps_;
		rclcpp::Subscription<OdomMSG>::SharedPtr sub_odom_origin_;

		rclcpp::Subscription<ImuMSG>::SharedPtr sub_imu_;

		void gps_ori_callback(const std::shared_ptr<GpsMSG> fix);
		void gps_filter_callback(const std::shared_ptr<GpsMSG> fix);
		void odom_origin_callback(const std::shared_ptr<OdomMSG> odom);
		void odom_gps_callback(const std::shared_ptr<OdomMSG> odom);
		void odom_global_callback(const std::shared_ptr<OdomMSG> odom);
		void imu_callback(const std::shared_ptr<ImuMSG> imu);
		bool flag_log=false;
		int log_num=0;
	public :
		GpsErrchk();


};
#endif
