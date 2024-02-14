/**
 * @file gps_to_slam_transformer.hpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief header file for gps_to_slam_transformer.cpp
 */

#ifndef GPS_TO_SLAM_TRANSFORMER__HPP
#define GPS_TO_SLAM_TRANSFORMER__HPP

#include "define_container/define_container.hpp"
#include "math/math.hpp"

/**
 * @namespace gps_to_slam_transformer
 * @brief namespace for define CommonMath, Transformer classes
 */

namespace gps_to_slam_transformer
{

    /**
     * @class gps_to_slam_transformer::Transformer
     * @brief final class for implements gps_to_slam_transformer by extends rclcpp::Node
     */
    class Transformer final : public rclcpp::Node
    {
    private:
        /**
         * @brief int field instance for gps_goal_waypoints_vec_'s index
         */
        int slam_goal_waypoints_vec_idx_;

        size_t slam_goal_waypoints_vec_size_;

        int slam_goal_waypoints_scenario_idx_;

        /**
         * @brief double field instance for gps_goal_waypoints
         */
        std::vector<geometry_msgs::msg::Pose> gps_goal_waypoints_vec_;

        std::vector<gps_to_slam_transformer::SLAMPoint> slam_goal_waypoints_vec_;

        /**
         * @brief int field instance for gps_goal_waypoints_vec_'s size
         */
        size_t gps_goal_waypoints_vec_size_;

        std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_ptr_;

        std::shared_ptr<gps_to_slam_transformer::GTSPoint> gts_point_ptr_;

        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_ptr_;

        std::shared_ptr<gps_to_slam_transformer::QuaternionPoint> quaternion_point_ptr_;

        /**
         * @brief shared pointer for gps_to_slam_transformer::CommonMath
         */
        std::shared_ptr<gps_to_slam_transformer::CommonMath> common_math_ptr_;

        std::shared_ptr<gps_to_slam_transformer::GTSMath> gts_math_ptr_;

        std::shared_ptr<gps_to_slam_transformer::GpsMath> gps_math_ptr_;

        std::shared_ptr<gps_to_slam_transformer::SLAMMath> slam_math_ptr_;

        std::shared_ptr<gps_to_slam_transformer::KTMMath> ktm_math_ptr_;

        std::shared_ptr<gps_to_slam_transformer::SLAMPointGenerator> slam_point_generator_ptr_;

        /**
         * @brief shared pointer for rclcpp::Node
         */
        rclcpp::Node::SharedPtr rcl_node_ptr_;

        /**
         * @brief shared pointer for rclcpp_action::Client<nav2_msgs::action::NavigateToPose>
         */
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr rcl_navigate_to_pose_action_client_ptr_;

        /**
         * @brief shared pointer for rclcpp::CallbackGroup which indicates rcl_ublox_fix_subscription_ptr_;
         */
        rclcpp::CallbackGroup::SharedPtr rcl_ublox_fix_subscription_callback_group_ptr_;

        /**
         * @brief shared pointer for rclcpp::Subscription<sensor_msgs::msg::NavSatFix>
         */
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rcl_ublox_fix_subscription_ptr_;

        /**
         * @brief shared pointer for rclcpp::CallbackGroup which indicates rcl_gps_goal_waypoints_stamped_ptr_;
         */
        rclcpp::CallbackGroup::SharedPtr rcl_gps_goal_waypoints_stamped_subscription_callback_group_ptr_;

        /**
         * @brief shared pointer for rclcpp::Subscription<gps_navigation_msgs::msg::GoalWayPointsStamped>
         */
        rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>::SharedPtr rcl_gps_goal_waypoints_stamped_subscription_ptr_;

        rclcpp::CallbackGroup::SharedPtr rcl_robot_pose_subscription_callback_group_ptr_;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr rcl_robot_pose_subscription_ptr_;

        rclcpp::CallbackGroup::SharedPtr rcl_navigation_status_stamped_publisher_callback_group_ptr_;

        rclcpp::Publisher<gps_navigation_msgs::msg::NavigationStatusStamped>::SharedPtr rcl_navigation_status_stamped_publisher_ptr_;

        rclcpp::CallbackGroup::SharedPtr rcl_navigation_result_stamped_publisher_callback_group_ptr_;

        rclcpp::Publisher<gps_navigation_msgs::msg::NavigationResultStamped>::SharedPtr rcl_navigation_result_stamped_publisher_ptr_;

        rclcpp::CallbackGroup::SharedPtr rcl_navigate_to_pose_goal_status_subscription_callback_group_ptr_;

        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr rcl_navigate_to_pose_goal_status_subscription_ptr_;

        rclcpp::Service<gps_navigation_msgs::srv::GoalCancel>::SharedPtr rcl_navigate_to_pose_cancel_goal_service_server_ptr_;

        rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr rcl_nav2_clear_global_costmap_service_client_ptr_;

        rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr rcl_nav2_clear_local_costmap_service_client_ptr_;

        /**
         * @brief function for flag established RCL connections
         * @return void
         */
        void flag_rcl_connections(const char *connection_type, const char *connection_name);

        /**
         * @brief function for handle subscription callback from rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>
         * @param gps_goal_waypoints_ptr const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr
         * @return void
         */
        void gps_goal_waypoints_callback(const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr gps_goal_waypoints_ptr);

        /**
         * @brief function for handle subscription callback from rclcpp::Subscription<sensor_msgs::msg::NavSatFix>
         * @param ublox_fix_ptr cosnt sensor_msgs::msg::NavSatFix::SharedPtr
         * @return void
         */
        void ublox_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_ptr);

        geometry_msgs::msg::Pose::SharedPtr rcl_robot_pose_ptr_;

        void robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr robot_pose_ptr);

        void nav2_clear_global_costmap_request();

        void nav2_clear_local_costmap_request();

        void navigation_status_stamped_publish(const gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &rcl_navigation_status_stamped_ptr, const int &goal_status_code);

        void navigation_result_stamped_publish(const gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr &rcl_navigation_result_stamped_ptr, const int &goal_status_code);

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr rcl_navigate_to_pose_goal_handle_ptr_;
        
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal::SharedPtr rcl_navigate_to_pose_goal_ptr_;

        void navigate_to_pose_goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_ptr);

        void navigate_to_pose_cancel_goal(const gps_navigation_msgs::srv::GoalCancel::Request::SharedPtr goal_cancel_request_ptr, gps_navigation_msgs::srv::GoalCancel::Response::SharedPtr goal_cancel_response_ptr);

        /**
         * @brief function for send goal to rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
         * @return void
         */
        void navigate_to_pose_send_goal();

        /**
         * @brief function for handle response callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
         * @param future std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>
         * @return void
         */
        void navigate_to_pose_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future);

        /**
         * @brief function for handle feedback callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
         * @param goal_handle_ptr const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
         * @param feedback_ptr const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
         * @return void
         */
        void navigate_to_pose_feedback_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ptr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ptr);

        /**
         * @brief function for handle result callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
         * @param result const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
         * @return void
         */
        void navigate_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    public:
        /**
         * create a new this class' instance
         * @brief default constructor
         */
        explicit Transformer();

        /**
         * destroy this class' instance
         * @brief default destructor
         */
        virtual ~Transformer();

        /**
         * @brief function for handle signal_input when program exit
         * @param signal_input The signal_input of input
         * @return void
         * @see signal_input.h
         */
        static void signal_handler(int signal_input);
    };
}

#endif