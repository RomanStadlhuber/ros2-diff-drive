#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <math.h>
#include "angular_vel/msg/diff_drive_omega.hpp"

#define PARAM_ANGULAR_ACCELERATION "angularacc"
#define EASE_MARGIN 0.1

using namespace std::chrono_literals;
using std::placeholders::_1; // used to signify placeholder parameters

// this file contains the two nodelets that publish and subscribe in one file

class WheelPublisher:public rclcpp::Node{
  public:
    WheelPublisher():Node("wheel_driver")
    {

      this->declare_parameter(PARAM_ANGULAR_ACCELERATION, 0.0);
      this->get_parameter(PARAM_ANGULAR_ACCELERATION,_angular_accelleration);

      RCLCPP_INFO(this->get_logger(), "initialize wheel driver with angular accelleration: %lf", _angular_accelleration);

      /* 
       * shared pointer to the nodelet
       * generic argument is message interface type
       * 1st parameter is topic name, 2nd is QoS Level
      */
      publisher_ = this -> create_publisher<angular_vel::msg::DiffDriveOmega>("angular_vel_out", 10);

      /** 
       * initialize ros publish timer
       */
      timer_ = this -> create_wall_timer(500ms, std::bind(&WheelPublisher::timer_callback, this));

      subscription_ = this -> create_subscription<angular_vel::msg::DiffDriveOmega>(
        "angular_vel_in",
        10,
        std::bind(&WheelPublisher::input_callback, this, _1)
      );

      /**
       * updates wheel velocity to target every 10 seconds
       * an incoming input is stored as target value and the time since last divert is tracked
       * the easing then happens as long as the velocity isn't within a certain error margin to the target value
       */
      ease_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&WheelPublisher::control_wheel_velocities, this)
      );

    } 

  private:

    void timer_callback(){
      auto message = angular_vel::msg::DiffDriveOmega();
      
      // for now, set both to 0.5 rad/s
      message.wl = _wl;
      message.wr = _wr;

      publisher_->publish(message);
    }


    void input_callback(angular_vel::msg::DiffDriveOmega::SharedPtr input){
      _wl_target = input -> wl;
      _wr_target = input -> wr;
      last_divert_ = nodeclock_ -> now();
    }

  /**
   * NOTE:
   * for now simulates linear acceleration
   */
    void control_wheel_velocities(){

      if(!is_in_margin(_wl,_wl_target, EASE_MARGIN)){

        RCLCPP_INFO(this->get_logger(), "divergenve in wl!");

        dt_l_ = nodeclock_ -> now() - last_divert_;

        _wl += (_wl > _wl_target?-1.0:1.0) * _angular_accelleration * dt_l_.seconds();
      }else{
        _wl = _wl_target;
      } 

      if(!is_in_margin(_wr,_wr_target, EASE_MARGIN)){

        RCLCPP_INFO(this->get_logger(), "divergenve in wr! ");

        dt_r_ = nodeclock_ -> now() - last_divert_;

        _wr += (_wr > _wr_target?-1.0:1.0) * _angular_accelleration * dt_r_.seconds();
      }else{
        _wr = _wr_target;
      }
    } 

    bool is_in_margin(double value, double target, double margin){
      return (abs(target-value) <= margin)? true:false;
    }
      

    // contain current angular velocity of the motors
    double _wl, _wr, _angular_accelleration, _wl_target, _wr_target = 0.0;

    rclcpp::Publisher<angular_vel::msg::DiffDriveOmega>::SharedPtr publisher_; // shared pointer to the nodelet
    rclcpp::TimerBase::SharedPtr timer_; // internal timer for publishing nodes
    rclcpp::Subscription<angular_vel::msg::DiffDriveOmega>::SharedPtr subscription_;

    rclcpp::Clock::SharedPtr nodeclock_ = this -> get_clock();

    /**
     * TODO: find way to accellerate wheels over time+
     * there only needs to be one tracking value for divert time
     * when a new input arrives one if the velocities or both might not fit
     * time deltas (see control funciton) are updated as long as a divergece occurs and is outside the margin
     */

    rclcpp::TimerBase::SharedPtr ease_timer_;
    rclcpp::Time last_divert_ = nodeclock_ -> now();
    rclcpp::Duration dt_l_ = rclcpp::Duration(0);
    rclcpp::Duration dt_r_ = rclcpp::Duration(0);
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelPublisher>());
  rclcpp::shutdown();
  return 0;
}
