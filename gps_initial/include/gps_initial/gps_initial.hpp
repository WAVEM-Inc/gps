#ifndef OBS_DETECTION_LISTENER_HPP__
#define OBS_DETECTION_LISTENER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "route_msgs/msg/drive_state.hpp"
#include <unistd.h>
using GpsMSG = sensor_msgs::msg::NavSatFix;
using DriveMSG = route_msgs::msg::DriveState;
class GpsInitial : public rclcpp::Node{
	private :
		rclcpp::CallbackGroup::SharedPtr cb_group_initgps_;
		rclcpp::Publisher<GpsMSG>::SharedPtr pub_initgps_;
		rclcpp::Subscription<DriveMSG>::SharedPtr sub_drive_;
		void drive_callback(const std::shared_ptr<DriveMSG> drive);
		double init_latitude=0;
		double init_longitude=0;

	public :
		GpsInitial();

};
#endif
