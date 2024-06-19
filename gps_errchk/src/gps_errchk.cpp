#include"gps_errchk.hpp"

GpsErrchk::GpsErrchk():Node("gps_errchk_node"){
	sub_ori_gps_ = this->create_subscription<GpsMSG>("/sensor/ublox/fix", 1, std::bind(&GpsErrchk::gps_ori_callback ,this ,std::placeholders::_1));

	sub_filter_gps_ = this->create_subscription<GpsMSG>("/sensor/ublox/fix_local", 1, std::bind(&GpsErrchk::gps_filter_callback ,this ,std::placeholders::_1));

	sub_odom_global_ = this->create_subscription<OdomMSG>("/odometry/global", 1, std::bind(&GpsErrchk::odom_global_callback ,this ,std::placeholders::_1));

	sub_odom_gps_ = this->create_subscription<OdomMSG>("/odometry/gps", 1, std::bind(&GpsErrchk::odom_gps_callback ,this ,std::placeholders::_1));

	sub_odom_origin_ = this->create_subscription<OdomMSG>("/drive/odom/origin", 1, std::bind(&GpsErrchk::odom_origin_callback ,this ,std::placeholders::_1));

	sub_imu_ = this->create_subscription<ImuMSG>("/sensor/imu/data", 1, std::bind(&GpsErrchk::imu_callback ,this ,std::placeholders::_1));
}

void GpsErrchk::gps_ori_callback(const std::shared_ptr<GpsMSG> fix)
{
	RCLCPP_INFO(this->get_logger(), "/sensor/ublox/fix lat=%lf\t,long=%lf\t,altitude=%lf\tstatus=%d\t,service=%d",fix->latitude,fix->longitude,fix->altitude,fix->status.status,fix->status.service );
}
void GpsErrchk::gps_filter_callback(const std::shared_ptr<GpsMSG> fix)
{
	RCLCPP_INFO(this->get_logger(), "/sensor/ublox/fix_local lat=%lf\t,long=%lf\t,altitude=%lf\tstatus=%d\t,service=%d",fix->latitude,fix->longitude,fix->altitude,fix->status.status,fix->status.service );
}

void GpsErrchk::odom_global_callback(const std::shared_ptr<OdomMSG> odom)
{
	RCLCPP_INFO(this->get_logger(), "/odometry/global	pose.position.x=%lf,\tpose.position.y=%lf,\t,pose.orientation.x,y,z,w=%lf,%lf,%lf,%lf,\ttwist.twist.linear,anguler=%lf,%lf",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w,odom->twist.twist.linear.x,odom->twist.twist.angular.z );
}
void GpsErrchk::odom_gps_callback(const std::shared_ptr<OdomMSG> odom)
{
	RCLCPP_INFO(this->get_logger(), "/odometry/gps	pose.position.x=%lf,\tpose.position.y=%lf,\t,pose.orientation.x,y,z,w=%lf,%lf,%lf,%lf,\ttwist.twist.linear,anguler=%lf,%lf",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w,odom->twist.twist.linear.x,odom->twist.twist.angular.z );
}
void GpsErrchk::odom_origin_callback(const std::shared_ptr<OdomMSG> odom)
{
	RCLCPP_INFO(this->get_logger(), "/drive/odom/origin	pose.position.x=%lf,\tpose.position.y=%lf,\t,pose.orientation.x,y,z,w=%lf,%lf,%lf,%lf,\ttwist.twist.linear,anguler=%lf,%lf",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w,odom->twist.twist.linear.x,odom->twist.twist.angular.z );
}
void GpsErrchk::imu_callback(const std::shared_ptr<ImuMSG> imu)
{
	RCLCPP_INFO(this->get_logger(), "/sensor/imu/data\torientation.x.y.z.w=%lf,%lf,%lf,%lf,\tanguler_velocity.x,y,z=%lf,%lf,%lf,\tlinear_acc.x.y.z=%lf,%lf,%lf",imu->orientation.x,imu->orientation.y,imu->orientation.w,imu->orientation.z,imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z,imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z );
}
