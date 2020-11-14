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

      // log received information
      RCLCPP_INFO(this -> get_logger(), "w-left: %lf, w-right: %lf", omega->wl, omega->wr);
      
      // calculate planar and angular velocity
      double vel_planar = calc_vel_planar(&omega->wl, &omega->wr);
      double vel_angular = calc_vel_angular(&omega->wl, &omega->wr, &_axislen);

      // log calculation results
      RCLCPP_INFO(this -> get_logger(), "dv: %lf, dw: %lf", vel_planar, vel_angular);
    }

    double calc_vel_angular(double * wl, double * wr, double * axislen){
      return (*wr - *wl)/(*axislen);
    }

    double calc_vel_planar(double * wl, double * wr){
      return (*wr + *wl)/2;
    }

    double _axislen = 0.12;

    rclcpp::Subscription<angular_vel::msg::DiffDriveOmega>::SharedPtr subscription_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
