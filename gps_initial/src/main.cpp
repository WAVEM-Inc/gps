#include"gps_initial.hpp"
int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsInitial>());
    rclcpp::shutdown();
    return 0;
}
