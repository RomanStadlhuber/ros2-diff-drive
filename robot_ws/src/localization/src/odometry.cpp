#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <math.h>
#include "angular_vel/msg/diff_drive_omega.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

      publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
  private:

    void sub_callback(const angular_vel::msg::DiffDriveOmega::SharedPtr omega){

      // log received information
      RCLCPP_INFO(this -> get_logger(), "w-left: %lf, w-right: %lf", omega->wl, omega->wr);
      
      // calculate planar and angular velocity
      _dv = calc_vel_planar(&omega->wl, &omega->wr);
      _dw = calc_vel_angular(&omega->wl, &omega->wr, &_axislen);

      // log calculation results
      RCLCPP_INFO(this -> get_logger(), "dv: %lf, dw: %lf", _dv, _dw);

      // ----------------- prepare twist message ------------------

      auto planar = geometry_msgs::msg::Vector3();

      planar.x = _dv * cos(_dw);
      planar.y = _dv * sin(_dw);
      planar.z = 0;

      auto angular = geometry_msgs::msg::Vector3();

      angular.x = angular.y = 0;

      angular.z = _dw;

      auto message = geometry_msgs::msg::Twist();

      message.angular = angular;
      message.linear = planar;

      //----------------- publish message -------------------------

      publisher_->publish(message);
    }

    double calc_vel_angular(double * wl, double * wr, double * axislen){
      return (*wr - *wl)/(*axislen);
    }

    double calc_vel_planar(double * wl, double * wr){
      return (*wr + *wl)/2;
    }

    double _axislen = 0.12;

    /**
     * define odometry buffer values
     */

    double _dv = 0.0; // instantaneous planar velocity "delta v"
    double _dw = 0.0; // instantaneous angular velocity "delta w"

    rclcpp::Subscription<angular_vel::msg::DiffDriveOmega>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
