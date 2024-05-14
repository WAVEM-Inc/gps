#include"gps_initial.hpp"

GpsInitial::GpsInitial():Node("gps_initial_node"){
	sub_drive_ = this->create_subscription<DriveMSG>("/drive/info", 1, std::bind(&GpsInitial::drive_callback ,this ,std::placeholders::_1));

	cb_group_initgps_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::PublisherOptions pub_initgps_options;
	pub_initgps_options.callback_group = cb_group_initgps_;
	pub_initgps_ = this->create_publisher<GpsMSG>("/sensor/ublox/initialfix", 1,pub_initgps_options);
}

void GpsInitial::drive_callback(const std::shared_ptr<DriveMSG> drive)
{
	GpsMSG gps;
	if(drive->code.compare(std::string("arrive"))==0)
	{
		//	init_latitude=drive->end_node.position.latitude;
		//	init_longitude=drive->end_node.position.longitude;
		if((drive->start_node.driving_option.compare(std::string("odom"))==0) || (drive->end_node.driving_option.compare(std::string("odom"))==0))
		{
			gps.latitude=drive->end_node.position.latitude;
			gps.longitude=drive->end_node.position.longitude;
			gps.altitude=100;
			pub_initgps_->publish(gps);
		}
	}
	else if(drive->start_node.driving_option.compare(std::string("odom"))==0)
	{
		gps.latitude=drive->start_node.position.latitude;
		gps.longitude=drive->start_node.position.longitude;
		gps.altitude=100;
		pub_initgps_->publish(gps);
	}	
}

