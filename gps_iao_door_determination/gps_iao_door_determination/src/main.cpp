#include <iostream>
#include "Determination.hpp"
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    std::shared_ptr<GPS::Determination> determination_run = std::make_shared<GPS::Determination>();
    rclcpp::spin(determination_run);
    rclcpp::shutdown();
    return 0;
}