/**
 * @file main.cpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief implementation file for main program
 */

/**
 * @brief include/gps_to_slam_transformer/gps_to_slam_transformer.hpp include area
 * @include gps_to_slam_transformer/gps_to_slam_transformer.hpp
 */

#include "gps_to_slam_transformer/gps_to_slam_transformer.hpp"

/**
 * @brief function for initialize rclcpp & execute main program
 * @param argc int
 * @param argv const char * const *
 * @return RCL_STOP_FLAG int
 */
int main(int argc, const char *const *argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr rcl_node_ptr = std::make_shared<gps_to_slam_transformer::Transformer>();

	signal(SIGINT, &gps_to_slam_transformer::Transformer::signal_handler);
	signal(SIGTSTP, &gps_to_slam_transformer::Transformer::signal_handler);

	rclcpp::spin(rcl_node_ptr);
	rclcpp::shutdown();

	return RCL_STOP_FLAG;
}