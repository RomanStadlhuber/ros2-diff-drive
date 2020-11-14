#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include "angular_vel/msg/diff_drive_omega.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1; // used to signify placeholder parameters

class Odometry: public rclcpp::Node{
  public:
    Odometry():Node("odometry"){
      subscription_ = this -> create_subscription<angular_vel::msg::DiffDriveOmega>(
        "angular_vel_out",
        10,
        // the last argument indicated the number of paremeters the bound function needs
        std::bind(&Odometry::sub_callback, this, _1)
      );
    }
  private:

    void sub_callback(const angular_vel::msg::DiffDriveOmega::SharedPtr omega){
      RCLCPP_INFO(this -> get_logger(), "w-left: %d, w-right: %d", omega->wl, omega->wr);
    } 

    rclcpp::Subscription<angular_vel::msg::DiffDriveOmega>::SharedPtr subscription_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
