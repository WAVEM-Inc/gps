#ifndef GPS_IAO_DOOR_DETERMINATION__DETERMINATION__HPP_
#define GPS_IAO_DOOR_DETERMINATION__DETERMINATION__HPP_
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gps_iao_door_msgs/msg/in_out_door.hpp"
#include "Constant/Constants.hpp"
#include <deque>
#include "GpsData.hpp"
using GpsMsg = sensor_msgs::msg::NavSatFix;
using IAOMsg = gps_iao_door_msgs::msg::InOutDoor;
namespace GPS{
    class Determination : public rclcpp::Node {
        public :
            Determination();
        private :
            rclcpp::Subscription<GpsMsg>::SharedPtr sub_ublox_;
            rclcpp::Publisher<IAOMsg>::SharedPtr pub_iao_;
            std::unique_ptr<Constants::Topic> constant_topic_;
            std::unique_ptr<Constants::DeterminationData> constants_data_;
            std::deque<GPS::Data> gps_deque_;
            
            int iao_check(const std::shared_ptr<GPS::Data> gps_data);

            void limited_size_deque(const std::shared_ptr<GPS::Data> gps_data);
            void sub_ublox_callback(const GpsMsg::SharedPtr gps);

    };
}
#endif