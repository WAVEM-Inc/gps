/**
 * @file gps_to_slam_transformer.cpp
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief implementation file for rclcpp::Node
 */

/**
 * @brief include/gps_to_slam_transformer/gps_to_slam_transformer.hpp include area
 * @include gps_to_slam_transformer/gps_to_slam_transformer.hpp
 */

#include "gps_to_slam_transformer/gps_to_slam_transformer.hpp"

/**
 * create a new this class' instance and extends rclcpp::Node
 * @brief default constructor
 * @see rclcpp::Node
 */
gps_to_slam_transformer::Transformer::Transformer()
    : Node(RCL_NODE_NAME),
      slam_goal_waypoints_vec_idx_(RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX),
      slam_goal_waypoints_scenario_idx_(RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX)
{
    slam_point_ptr_ = std::make_shared<gps_to_slam_transformer::SLAMPoint>();
    gts_point_ptr_ = std::make_shared<gps_to_slam_transformer::GTSPoint>();
    ktm_point_ptr_ = std::make_shared<gps_to_slam_transformer::KTMPoint>();
    quaternion_point_ptr_ = std::make_shared<gps_to_slam_transformer::QuaternionPoint>();

    common_math_ptr_ = std::make_shared<gps_to_slam_transformer::CommonMath>();
    gts_math_ptr_ = std::make_shared<gps_to_slam_transformer::GTSMath>();
    gps_math_ptr_ = std::make_shared<gps_to_slam_transformer::GpsMath>();
    slam_math_ptr_ = std::make_shared<gps_to_slam_transformer::SLAMMath>();
    ktm_math_ptr_ = std::make_shared<gps_to_slam_transformer::KTMMath>();

    slam_point_generator_ptr_ = std::make_shared<gps_to_slam_transformer::SLAMPointGenerator>();

    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "gps_to_slam_transformer has been started...");
    RCLCPP_LINE_INFO();

    rcl_navigate_to_pose_action_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        rcl_node_ptr_,
        RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->flag_rcl_connections(RCL_ACTION_CLIENT_FLAG, RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    rcl_ublox_fix_subscription_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rcl_ublox_fix_subscription_options;
    rcl_ublox_fix_subscription_options.callback_group = rcl_ublox_fix_subscription_callback_group_ptr_;

    rcl_ublox_fix_subscription_ptr_ = rcl_node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_UBLOX_FIX_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_to_slam_transformer::Transformer::ublox_fix_callback, this, _1),
        rcl_ublox_fix_subscription_options);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_UBLOX_FIX_TOPIC);

    rcl_gps_goal_waypoints_stamped_subscription_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rcl_gps_goal_waypoints_subscription_options;
    rcl_gps_goal_waypoints_subscription_options.callback_group = rcl_gps_goal_waypoints_stamped_subscription_callback_group_ptr_;

    rcl_gps_goal_waypoints_stamped_subscription_ptr_ = rcl_node_ptr_->create_subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>(
        RCL_GPS_GOAL_WAYPOINTS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_to_slam_transformer::Transformer::gps_goal_waypoints_callback, this, _1),
        rcl_gps_goal_waypoints_subscription_options);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GPS_GOAL_WAYPOINTS_STAMPED_TOPIC);

    rcl_robot_pose_subscription_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rcl_robot_pose_subscription_options;
    rcl_robot_pose_subscription_options.callback_group = rcl_robot_pose_subscription_callback_group_ptr_;

    rcl_robot_pose_subscription_ptr_ = rcl_node_ptr_->create_subscription<geometry_msgs::msg::Pose>(
        RCL_ROBOT_POSE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_to_slam_transformer::Transformer::robot_pose_callback, this, _1),
        rcl_robot_pose_subscription_options);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_ROBOT_POSE_TOPIC);

    rcl_navigation_status_stamped_publisher_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rcl_navigation_status_stamped_publisher_options;
    rcl_navigation_status_stamped_publisher_options.callback_group = rcl_navigate_to_pose_goal_status_subscription_callback_group_ptr_;

    rcl_navigation_status_stamped_publisher_ptr_ = rcl_node_ptr_->create_publisher<gps_navigation_msgs::msg::NavigationStatusStamped>(
        RCL_NAVIGATION_STATUS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_STATUS_STAMPED_TOPIC);

    rcl_navigation_result_stamped_publisher_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rcl_navigation_result_stamped_publisher_options;
    rcl_navigation_result_stamped_publisher_options.callback_group = rcl_navigation_result_stamped_publisher_callback_group_ptr_;

    rcl_navigation_result_stamped_publisher_ptr_ = rcl_node_ptr_->create_publisher<gps_navigation_msgs::msg::NavigationResultStamped>(
        RCL_NAVIGATION_RESULT_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_RESULT_STAMPED_TOPIC);

    rcl_navigate_to_pose_goal_ptr_ = std::make_shared<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal>();

    rcl_navigate_to_pose_goal_status_subscription_callback_group_ptr_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rcl_navigate_to_pose_goal_status_subscription_options;
    rcl_navigate_to_pose_goal_status_subscription_options.callback_group = rcl_navigate_to_pose_goal_status_subscription_callback_group_ptr_;

    rcl_navigate_to_pose_goal_status_subscription_ptr_ = rcl_node_ptr_->create_subscription<action_msgs::msg::GoalStatusArray>(
        RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_to_slam_transformer::Transformer::navigate_to_pose_goal_status_callback, this, _1),
        rcl_navigate_to_pose_goal_status_subscription_options);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC);

    rcl_navigate_to_pose_cancel_goal_service_server_ptr_ = rcl_node_ptr_->create_service<gps_navigation_msgs::srv::GoalCancel>(
        RCL_NAVIGATE_TO_POSE_CANCEL_GOAL_SERVICE_SERVER_NAME,
        std::bind(&gps_to_slam_transformer::Transformer::navigate_to_pose_cancel_goal, this, _1, _2));

    this->flag_rcl_connections(RCL_SERVICE_SERVER_FLAG, RCL_NAVIGATE_TO_POSE_CANCEL_GOAL_SERVICE_SERVER_NAME);

    rcl_nav2_clear_global_costmap_service_client_ptr_ = rcl_node_ptr_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        RCL_NAV2_CLEAR_GLOBAL_COSTMAP_SERVICE_SERVER_NAME);

    this->flag_rcl_connections(RCL_SERVICE_CLIENT_FLAG, RCL_NAV2_CLEAR_GLOBAL_COSTMAP_SERVICE_SERVER_NAME);

    rcl_nav2_clear_local_costmap_service_client_ptr_ = rcl_node_ptr_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        RCL_NAV2_CLEAR_LOCAL_COSTMAP_SERVICE_SERVER_NAME);

    this->flag_rcl_connections(RCL_SERVICE_CLIENT_FLAG, RCL_NAV2_CLEAR_LOCAL_COSTMAP_SERVICE_SERVER_NAME);
}

/**
 * destroy this class' instance
 * @brief default destructor
 */
gps_to_slam_transformer::Transformer::~Transformer()
{
}

/**
 * @brief function for handle signal_input when program exit
 * @param signal_input The signal_input of input
 * @return void
 * @see signal_input.h
 */
void gps_to_slam_transformer::Transformer::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== gps_to_slam_transformer has been terminated with SIG [%d] =====", signal_input);
    signal(signal_input, SIG_IGN);
    exit(RCL_STOP_FLAG);
}

/**
 * @brief function for flag established RCL connections
 * @return void
 */
void gps_to_slam_transformer::Transformer::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "RCL [%s - %s] created...", connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

/**
 * @brief function for handle subscription callback from rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>
 * @param gps_goal_waypoints_ptr const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr
 * @return void
 */
void gps_to_slam_transformer::Transformer::gps_goal_waypoints_callback(const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr gps_goal_waypoints_ptr)
{
    bool is_gps_goal_waypoints_empty = gps_goal_waypoints_vec_.empty();

    if (!is_gps_goal_waypoints_empty)
    {
        slam_goal_waypoints_vec_idx_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
        slam_goal_waypoints_vec_size_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;

        gps_goal_waypoints_vec_.clear();
        slam_goal_waypoints_vec_.clear();

        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "===== gps_goal_waypoints has been renewed =====\n\twaypoints_vec_idx : [%d]\n\twaypoints_vec_size : [%d]",
            slam_goal_waypoints_vec_idx_,
            slam_goal_waypoints_vec_size_);
        RCLCPP_LINE_WARN();
    }

    gps_goal_waypoints_vec_ = gps_goal_waypoints_ptr->goal_waypoints_list;
    gps_goal_waypoints_vec_size_ = gps_goal_waypoints_vec_.size();

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "gps_waypoints_list schedule size : [%i]", gps_goal_waypoints_vec_size_);
    RCLCPP_LINE_INFO();

    for (const geometry_msgs::msg::Pose &gps_goal_waypoints : gps_goal_waypoints_vec_)
    {
        const double &gps_goal_x = gps_goal_waypoints.position.x;
        const double &gps_goal_y = gps_goal_waypoints.position.y;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(), "gps_waypoints_list vector\n\tx : [%f]\n\ty : [%f]",
            gps_goal_x,
            gps_goal_y);
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_ptr = ktm_math_ptr_->geo_point_to_ktm_point(
            gps_goal_x,
            gps_goal_y);

        std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_ptr = slam_point_generator_ptr_->geo_point_to_slam_point(gts_point_ptr_, ktm_point_ptr);
        slam_goal_waypoints_vec_.push_back(*slam_point_ptr);
    }

    for (gps_to_slam_transformer::SLAMPoint &slam_goal : slam_goal_waypoints_vec_)
    {
        const double &slam_goal_x = slam_goal.get__x();
        const double &slam_goal_y = slam_goal.get__y();

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "gps_waypoints_list slam points\n\tx : [%f]\n\ty : [%f]",
            slam_goal_x,
            slam_goal_y);
        RCLCPP_LINE_INFO();
    }

    gps_goal_waypoints_vec_ = gps_goal_waypoints_ptr->goal_waypoints_list;
    gps_goal_waypoints_vec_size_ = gps_goal_waypoints_vec_.size();
    slam_goal_waypoints_vec_size_ = gps_goal_waypoints_vec_size_;

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "gps_waypoints_list schedule size : [%i]", gps_goal_waypoints_vec_size_);
    RCLCPP_LINE_INFO();

    for (const geometry_msgs::msg::Pose &gps_goal_waypoints : gps_goal_waypoints_vec_)
    {
        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(), "gps_waypoints_list vector\n\tx : [%f]\n\ty : [%f]",
            gps_goal_waypoints.position.x,
            gps_goal_waypoints.position.y);
        RCLCPP_LINE_INFO();
    }

    this->navigate_to_pose_send_goal();
}

void gps_to_slam_transformer::Transformer::ublox_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_ptr)
{
    const double &current_gps_lat = ublox_fix_ptr->latitude;
    const double &current_gps_lon = ublox_fix_ptr->longitude;

    bool is_geo_point_1_ptr_default = (gts_point_ptr_->get__ktm_point_1_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point_ptr_->get__ktm_point_1_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
    bool is_geo_point_2_ptr_default = (gts_point_ptr_->get__ktm_point_2_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point_ptr_->get__ktm_point_2_ptr()->get__y() == GTS_DEFAULT_DOUBLE);

    bool is_robot_pose_ptr_null = rcl_robot_pose_ptr_ == nullptr;

    if (is_geo_point_1_ptr_default && !is_robot_pose_ptr_null)
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "ublox fix callback set first coordinate");
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_1_ptr = ktm_math_ptr_->geo_point_to_ktm_point(
            37.465932,
            127.124136);

        gts_point_ptr_->set__ktm_point_1_ptr(ktm_point_1_ptr);

        std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_1_ptr = gts_point_ptr_->get__slam_point_1_ptr();

        const double &robot_pose_x = rcl_robot_pose_ptr_->position.x;
        const double &robot_pose_y = rcl_robot_pose_ptr_->position.y;

        slam_point_1_ptr->set__x(2.724780);
        slam_point_1_ptr->set__y(-2.330876);

        gts_point_ptr_->set__slam_point_1_ptr(slam_point_1_ptr);

        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_current_point_ptr = ktm_math_ptr_->geo_point_to_ktm_point(
            current_gps_lat, current_gps_lon);

        const double &differ_1 = common_math_ptr_->distance_formula(
            gts_point_ptr_->get__ktm_point_1_ptr()->get__x(), gts_point_ptr_->get__ktm_point_1_ptr()->get__y(),
            ktm_current_point_ptr->get__x(), ktm_current_point_ptr->get__y());

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback KtS target result 1\n\tx : [%f]\n\ty : [%f]",
            ktm_point_1_ptr->get__x(),
            ktm_point_1_ptr->get__y());
        RCLCPP_LINE_INFO();

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback KtS result 1\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
            ktm_current_point_ptr->get__x(),
            ktm_current_point_ptr->get__y(),
            differ_1);
        RCLCPP_LINE_INFO();
    }

    if (!is_geo_point_1_ptr_default && is_geo_point_2_ptr_default)
    {
        const double &ktm_point_1_lat = gts_point_ptr_->get__ktm_point_1_ptr()->get__x();
        const double &ktm_point_1_lon = gts_point_ptr_->get__ktm_point_1_ptr()->get__y();

        const double &gps_distance = gps_math_ptr_->distance_formula_meters(
            ktm_point_1_lat,
            ktm_point_1_lon,
            current_gps_lat,
            current_gps_lon);

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback ktm point 1 gps distance : [%f]",
            gps_distance);
        RCLCPP_LINE_INFO();

        if (gps_distance >= 15)
        {
            std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_2_ptr = ktm_math_ptr_->geo_point_to_ktm_point(
                37.465798,
                127.124107);

            gts_point_ptr_->set__ktm_point_2_ptr(ktm_point_2_ptr);

            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_2_ptr = gts_point_ptr_->get__slam_point_2_ptr();

            const double &robot_pose_x = rcl_robot_pose_ptr_->position.x;
            const double &robot_pose_y = rcl_robot_pose_ptr_->position.y;

            slam_point_2_ptr->set__x(-12.386119);
            slam_point_2_ptr->set__y(0.497768);

            gts_point_ptr_->set__slam_point_2_ptr(slam_point_2_ptr);

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "======= ublox fix callback distance result =======\n\tKTM 1 lat : [%f]\n\tKTM 1 long : [%f]\n\tKTM 2 lat : [%f]\n\tKTM 2 long : [%f]\n\tSLAM 1 x : [%f]\n\tSLAM 1 y : [%f]\n\tSLAM 2 x : [%f]\n\tSLAM 2 y : [%f]",
                gts_point_ptr_->get__ktm_point_1_ptr()->get__x(),
                gts_point_ptr_->get__ktm_point_1_ptr()->get__y(),
                gts_point_ptr_->get__ktm_point_2_ptr()->get__x(),
                gts_point_ptr_->get__ktm_point_2_ptr()->get__y(),
                gts_point_ptr_->get__slam_point_1_ptr()->get__x(),
                gts_point_ptr_->get__slam_point_1_ptr()->get__y(),
                gts_point_ptr_->get__slam_point_2_ptr()->get__x(),
                gts_point_ptr_->get__slam_point_2_ptr()->get__y());
            RCLCPP_LINE_INFO();

            std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_current_point_ptr = ktm_math_ptr_->geo_point_to_ktm_point(
                current_gps_lat, current_gps_lon);

            const double &differ_2 = common_math_ptr_->distance_formula(
                gts_point_ptr_->get__ktm_point_2_ptr()->get__x(), gts_point_ptr_->get__ktm_point_2_ptr()->get__y(),
                ktm_current_point_ptr->get__x(), ktm_current_point_ptr->get__y());

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "ublox fix callback KtS target result 2\n\tx : [%f]\n\ty : [%f]",
                ktm_point_2_ptr->get__x(),
                ktm_point_2_ptr->get__y());
            RCLCPP_LINE_INFO();

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "ublox fix callback KtS result 2\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
                ktm_current_point_ptr->get__x(),
                ktm_current_point_ptr->get__y(),
                differ_2);
            RCLCPP_LINE_INFO();
        }
    }
}

void gps_to_slam_transformer::Transformer::robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr robot_pose_ptr)
{
    rcl_robot_pose_ptr_ = robot_pose_ptr;

    bool is_rcl_robot_pose_ptr_nullptr = rcl_robot_pose_ptr_ == nullptr;

    if (is_rcl_robot_pose_ptr_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "robot pose callback rcl_robot_pose_ptr is nullptr");
        RCLCPP_LINE_ERROR();
    }
}

void gps_to_slam_transformer::Transformer::navigate_to_pose_cancel_goal(
    const gps_navigation_msgs::srv::GoalCancel::Request::SharedPtr goal_cancel_request_ptr,
    gps_navigation_msgs::srv::GoalCancel::Response::SharedPtr goal_cancel_response_ptr)
{
    const bool &is_cancel_goal_current = goal_cancel_request_ptr->cancel_goal_current;
    const bool &is_cancel_goal_before = goal_cancel_request_ptr->cancel_goal_before;
    const bool &is_cancel_goal_all = goal_cancel_request_ptr->cancel_goal_all;

    if (is_cancel_goal_current)
    {
        std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr> rcl_navigate_to_pose_cancel_goal_future = rcl_navigate_to_pose_action_client_ptr_->async_cancel_goal(rcl_navigate_to_pose_goal_handle_ptr_);
        std::shared_ptr<action_msgs::srv::CancelGoal_Response> rcl_navigate_to_pose_cancel_goal_response_ptr = rcl_navigate_to_pose_cancel_goal_future.get();
        const int8_t &rcl_navigate_to_pose_cancel_goal_result = rcl_navigate_to_pose_cancel_goal_response_ptr->return_code;

        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "===== canceled navigate_to_pose's current goal with return code : [%d] =====",
            rcl_navigate_to_pose_cancel_goal_result);
        RCLCPP_LINE_WARN();
    }
    else if (is_cancel_goal_before)
    {
        const rclcpp::Time &rcl_time_now = rcl_node_ptr_->now();
        std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr> rcl_navigate_to_pose_cancel_goals_before_future = rcl_navigate_to_pose_action_client_ptr_->async_cancel_goals_before(rcl_time_now);
        std::shared_ptr<action_msgs::srv::CancelGoal_Response> rcl_navigate_to_pose_cancel_goals_before_response_ptr = rcl_navigate_to_pose_cancel_goals_before_future.get();
        const int8_t &rcl_navigate_to_pose_cancel_goals_before_result = rcl_navigate_to_pose_cancel_goals_before_response_ptr->return_code;

        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "===== canceled navigate_to_pose's before goals with return code : [%d] =====",
            rcl_navigate_to_pose_cancel_goals_before_result);
        RCLCPP_LINE_WARN();
    }
    else if (is_cancel_goal_all)
    {
        std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr> rcl_navigate_to_pose_cancel_all_goals_future = rcl_navigate_to_pose_action_client_ptr_->async_cancel_all_goals();
        std::shared_ptr<action_msgs::srv::CancelGoal_Response> rcl_navigate_to_pose_cancel_all_goals_response_ptr = rcl_navigate_to_pose_cancel_all_goals_future.get();
        const int8_t &rcl_navigate_to_pose_cancel_all_goals_result = rcl_navigate_to_pose_cancel_all_goals_response_ptr->return_code;

        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "===== canceled navigate_to_pose's goals all with return code : [%d] =====",
            rcl_navigate_to_pose_cancel_all_goals_result);
        RCLCPP_LINE_WARN();
    }
    else
    {
        return;
    }

    goal_cancel_response_ptr->set__is_goal_canceled(true);
}

void gps_to_slam_transformer::Transformer::nav2_clear_global_costmap_request()
{
    nav2_msgs::srv::ClearEntireCostmap_Request::SharedPtr nav2_clear_global_costmap_request_ptr = std::make_shared<nav2_msgs::srv::ClearEntireCostmap_Request>();

    bool is_nav2_clear_global_costmap_service_server_ready = rcl_nav2_clear_global_costmap_service_client_ptr_->wait_for_service(std::chrono::seconds(1));

    if (!is_nav2_clear_global_costmap_service_server_ready)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "nav2 clear global costmap service server is not ready...");
        RCLCPP_LINE_ERROR();
        return;
    }
    else
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== nav2 clearing global costmap =====");
        RCLCPP_LINE_INFO();
        rcl_nav2_clear_global_costmap_service_client_ptr_->async_send_request(nav2_clear_global_costmap_request_ptr);
    }
}

void gps_to_slam_transformer::Transformer::nav2_clear_local_costmap_request()
{
    nav2_msgs::srv::ClearEntireCostmap_Request::SharedPtr nav2_clear_local_costmap_request_ptr = std::make_shared<nav2_msgs::srv::ClearEntireCostmap_Request>();

    bool is_nav2_clear_local_costmap_service_server_ready = rcl_nav2_clear_local_costmap_service_client_ptr_->wait_for_service(std::chrono::seconds(1));

    if (!is_nav2_clear_local_costmap_service_server_ready)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "nav2 clear locbal costmap service server is not ready...");
        RCLCPP_LINE_ERROR();
        return;
    }
    else
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== nav2 clearing local costmap =====");
        RCLCPP_LINE_INFO();
        rcl_nav2_clear_local_costmap_service_client_ptr_->async_send_request(nav2_clear_local_costmap_request_ptr);
    }
}

void gps_to_slam_transformer::Transformer::navigation_status_stamped_publish(const gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &rcl_navigation_status_stamped_ptr, const int &goal_status_code)
{
    bool is_rcl_navigation_status_stamped_ptr_nullptr = rcl_navigation_status_stamped_ptr == nullptr;

    if (is_rcl_navigation_status_stamped_ptr_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigation_status_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigation_status_stamped published");
    RCLCPP_LINE_INFO();

    rcl_navigation_status_stamped_ptr->set__status_code(goal_status_code);
    rcl_navigation_status_stamped_ptr->set__current_goal_index(slam_goal_waypoints_vec_idx_);

    const gps_navigation_msgs::msg::NavigationStatusStamped &&rcl_navigation_status_stamped_ptr_moved = std::move(*rcl_navigation_status_stamped_ptr);
    rcl_navigation_status_stamped_publisher_ptr_->publish(rcl_navigation_status_stamped_ptr_moved);
}

void gps_to_slam_transformer::Transformer::navigation_result_stamped_publish(const gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr &rcl_navigation_result_stamped_ptr, const int &goal_status_code)
{
    bool is_rcl_navigation_result_stamped_ptr_nullptr = rcl_navigation_result_stamped_ptr == nullptr;

    if (is_rcl_navigation_result_stamped_ptr_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigation_result_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigation_resykt_stamped published");
    RCLCPP_LINE_INFO();

    rcl_navigation_result_stamped_ptr->set__result_code(goal_status_code);
    rcl_navigation_result_stamped_ptr->set__result_index(slam_goal_waypoints_scenario_idx_);

    const gps_navigation_msgs::msg::NavigationResultStamped &&rcl_navigation_result_stamped_ptr_moved = std::move(*rcl_navigation_result_stamped_ptr);
    rcl_navigation_result_stamped_publisher_ptr_->publish(rcl_navigation_result_stamped_ptr_moved);
}

void gps_to_slam_transformer::Transformer::navigate_to_pose_goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_ptr)
{
    bool is_goal_status_array_empty = goal_status_array_ptr->status_list.empty();

    if (is_goal_status_array_empty)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose goal status callback status list is empty...");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
        std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

        const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
        const int32_t &current_time_nanosec = current_time_sec % 1000000000;

        builtin_interfaces::msg::Time::UniquePtr rcl_builtin_interfaces_time_ptr = std::make_unique<builtin_interfaces::msg::Time>();
        rcl_builtin_interfaces_time_ptr->set__sec(current_time_sec);
        rcl_builtin_interfaces_time_ptr->set__nanosec(current_time_nanosec);

        std_msgs::msg::Header::UniquePtr rcl_std_msgs_header_ptr = std::make_unique<std_msgs::msg::Header>();
        rcl_std_msgs_header_ptr->set__frame_id(RCL_HEADER_FRAME_ID);

        const builtin_interfaces::msg::Time &&rcl_builtin_interfaces_time_ptr_moved = std::move(*rcl_builtin_interfaces_time_ptr);
        rcl_std_msgs_header_ptr->set__stamp(rcl_builtin_interfaces_time_ptr_moved);

        const std_msgs::msg::Header &&rcl_std_msgs_header_ptr_moved = std::move(*rcl_std_msgs_header_ptr);

        gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr rcl_navigation_status_stamped_ptr = std::make_unique<gps_navigation_msgs::msg::NavigationStatusStamped>();
        rcl_navigation_status_stamped_ptr->set__header(rcl_std_msgs_header_ptr_moved);

        gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr rcl_navigation_result_stamped_ptr = std::make_unique<gps_navigation_msgs::msg::NavigationResultStamped>();
        rcl_navigation_result_stamped_ptr->set__header(rcl_std_msgs_header_ptr_moved);

        const action_msgs::msg::GoalStatus &goal_status = goal_status_array_ptr->status_list.back();
        const uint8_t &goal_status_code = goal_status.status;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose status callback\n\twaypoints index : [%d]\n\twaypoints size : [%d]\n\tgoal_status_code : [%d]",
            slam_goal_waypoints_vec_idx_ + 1,
            slam_goal_waypoints_vec_size_,
            goal_status_code);
        RCLCPP_LINE_INFO();

        bool is_slam_goal_waypoints_ended = slam_goal_waypoints_vec_idx_ == (slam_goal_waypoints_vec_size_ - 1);

        if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED)
        {
            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal started =====");
            RCLCPP_LINE_INFO();

            this->navigation_status_stamped_publish(rcl_navigation_status_stamped_ptr, goal_status_code);
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED)
        {
            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal succeeded =====");
            RCLCPP_LINE_INFO();

            if (is_slam_goal_waypoints_ended)
            {
                RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal waypoints ended =====");
                RCLCPP_LINE_INFO();

                slam_goal_waypoints_scenario_idx_++;

                this->navigation_result_stamped_publish(rcl_navigation_result_stamped_ptr, goal_status_code);

                slam_goal_waypoints_vec_idx_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
                slam_goal_waypoints_vec_size_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;

                gps_goal_waypoints_vec_.clear();
                slam_goal_waypoints_vec_.clear();

                rcl_navigate_to_pose_action_client_ptr_->async_cancel_all_goals();
            }
            else
            {
                slam_goal_waypoints_vec_idx_++;

                RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback will proceed next [%d] goal =====", slam_goal_waypoints_vec_idx_);
                RCLCPP_LINE_INFO();

                this->navigation_status_stamped_publish(rcl_navigation_status_stamped_ptr, goal_status_code);
                this->navigate_to_pose_send_goal();
            }
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_ABORTED)
        {
            RCLCPP_WARN(rcl_node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal aborted !!!!!");
            RCLCPP_LINE_WARN();

            RCLCPP_WARN(rcl_node_ptr_->get_logger(), "navigate_to_pose status callback will proceed previous [%d]st goal", slam_goal_waypoints_vec_idx_);
            RCLCPP_LINE_WARN();

            this->navigation_status_stamped_publish(rcl_navigation_status_stamped_ptr, goal_status_code);

            // this->nav2_clear_global_costmap_request();
            // this->nav2_clear_local_costmap_request();
            this->navigate_to_pose_send_goal();
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_CANCELED)
        {
            RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal canceled !!!!!");
            RCLCPP_LINE_ERROR();

            this->navigation_status_stamped_publish(rcl_navigation_status_stamped_ptr, goal_status_code);
        }
        else
        {
            RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose status callback unknown goal result code : [%d]", goal_status_code);
            RCLCPP_LINE_ERROR();
            return;
        }
    }
}

/**
 * @brief function for send goal to rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @return void
 */
void gps_to_slam_transformer::Transformer::navigate_to_pose_send_goal()
{
    bool is_slam_goal_waypoints_vec_empty = slam_goal_waypoints_vec_.empty();

    if (is_slam_goal_waypoints_vec_empty)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose send goal waypoints is empty...");
        RCLCPP_LINE_ERROR();
        return;
    }

    const double &slam_x = slam_goal_waypoints_vec_[slam_goal_waypoints_vec_idx_].get__x();
    const double &slam_y = slam_goal_waypoints_vec_[slam_goal_waypoints_vec_idx_].get__y();

    bool is_robot_pose_ptr_nullptr = rcl_robot_pose_ptr_ == nullptr;

    if (!is_robot_pose_ptr_nullptr)
    {
        const double &current_x = rcl_robot_pose_ptr_->position.x;
        const double &current_y = rcl_robot_pose_ptr_->position.y;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose send goal\n\tcurrent x: : [%f]\n\tcurrent y : [%f]\n\tslam x : [%f]\n\tslam y : [%f]",
            current_x,
            current_y,
            slam_x,
            slam_y);
        RCLCPP_LINE_INFO();

        const double &angle = common_math_ptr_->angle_between_two_points(current_x, current_y, slam_x, slam_y);

        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigate_to_pose send goal angle : [%f]", angle);
        RCLCPP_LINE_INFO();

        quaternion_point_ptr_->euler_angle_to_quaternion(angle, RCL_DEFAULT_DOUBLE, RCL_DEFAULT_DOUBLE);

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose send goal quaternion\n\tz : [%f]\n\tw : [%f]",
            quaternion_point_ptr_->get__euler_z(),
            quaternion_point_ptr_->get__euler_w());
        RCLCPP_LINE_INFO();
    }

    bool is_rcl_navigate_to_pose_action_server_ready = rcl_navigate_to_pose_action_client_ptr_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_rcl_navigate_to_pose_action_server_ready)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose action server is not available after waiting");
        RCLCPP_LINE_ERROR();
        return;
    }
    else
    {
        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose action server is ready\n\tslam x : [%f]\n\tslam y : [%f]",
            slam_x,
            slam_y);
        RCLCPP_LINE_INFO();
    }

    geometry_msgs::msg::PoseStamped::UniquePtr rcl_pose_stamped_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>();

    geometry_msgs::msg::Point::UniquePtr rcl_point_ptr = std::make_unique<geometry_msgs::msg::Point>();
    rcl_point_ptr->set__x(slam_x);
    rcl_point_ptr->set__y(slam_y);
    rcl_point_ptr->set__z(RCL_DEFAULT_DOUBLE);

    geometry_msgs::msg::Quaternion::UniquePtr rcl_quaternion_ptr = std::make_unique<geometry_msgs::msg::Quaternion>();
    rcl_quaternion_ptr->set__x(RCL_DEFAULT_DOUBLE);
    rcl_quaternion_ptr->set__y(RCL_DEFAULT_DOUBLE);
    rcl_quaternion_ptr->set__z(quaternion_point_ptr_->get__euler_z());
    rcl_quaternion_ptr->set__w(quaternion_point_ptr_->get__euler_w());

    geometry_msgs::msg::Pose::UniquePtr rcl_pose_ptr = std::make_unique<geometry_msgs::msg::Pose>();

    const geometry_msgs::msg::Point &&rcl_point_ptr_moved = std::move(*rcl_point_ptr);
    rcl_pose_ptr->set__position(rcl_point_ptr_moved);

    const geometry_msgs::msg::Quaternion &&rcl_quaternion_ptr_moved = std::move(*rcl_quaternion_ptr);
    rcl_pose_ptr->set__orientation(rcl_quaternion_ptr_moved);

    const geometry_msgs::msg::Pose &&rcl_pose_ptr_moved = std::move(*rcl_pose_ptr);
    rcl_pose_stamped_ptr->set__pose(rcl_pose_ptr_moved);

    const geometry_msgs::msg::PoseStamped &&rcl_pose_stamped_ptr_moved = std::move(*rcl_pose_stamped_ptr);
    rcl_navigate_to_pose_goal_ptr_->set__pose(rcl_pose_stamped_ptr_moved);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions rcl_navigate_to_pose_send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    rcl_navigate_to_pose_send_goal_options.feedback_callback = std::bind(&gps_to_slam_transformer::Transformer::navigate_to_pose_feedback_callback, this, _1, _2);
    rcl_navigate_to_pose_send_goal_options.goal_response_callback = std::bind(&gps_to_slam_transformer::Transformer::navigate_to_pose_goal_response_callback, this, _1);
    rcl_navigate_to_pose_send_goal_options.result_callback = std::bind(&gps_to_slam_transformer::Transformer::navigate_to_pose_result_callback, this, _1);

    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> rcl_navigate_to_pose_goal_future = rcl_navigate_to_pose_action_client_ptr_->async_send_goal(*rcl_navigate_to_pose_goal_ptr_, rcl_navigate_to_pose_send_goal_options);
    rcl_navigate_to_pose_goal_handle_ptr_ = rcl_navigate_to_pose_goal_future.get();

    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(),
        "navigate_to_pose goal sent\n\tpose_x : [%f]\n\tpose_y : [%f]\n\torien_z: [%f]\n\torien_w : [%f]",
        rcl_navigate_to_pose_goal_ptr_->pose.pose.position.x,
        rcl_navigate_to_pose_goal_ptr_->pose.pose.position.y,
        rcl_navigate_to_pose_goal_ptr_->pose.pose.orientation.z,
        rcl_navigate_to_pose_goal_ptr_->pose.pose.orientation.w);
    RCLCPP_LINE_INFO();
}

/**
 * @brief function for handle response callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param future std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>
 * @return void
 */
void gps_to_slam_transformer::Transformer::navigate_to_pose_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
{
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle = future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose goal response callback was rejected by [%s] server", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigate_to_pose goal response callback has been accepted by [%s] server, waiting for result...", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_INFO();
    }
}

/**
 * @brief function for handle feedback callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param goal_handle_ptr const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
 * @param feedback_ptr const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
 * @return void
 */
void gps_to_slam_transformer::Transformer::navigate_to_pose_feedback_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ptr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ptr)
{
}

/**
 * @brief function for handle result callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param result const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
 * @return void
 */
void gps_to_slam_transformer::Transformer::navigate_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result)
{
}