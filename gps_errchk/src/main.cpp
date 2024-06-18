#include"gps_errchk.hpp"
int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsErrchk>());
    rclcpp::shutdown();
    return 0;
}
