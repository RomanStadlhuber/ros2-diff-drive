#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include "angular_vel/msg/diff_drive_omega.hpp"

using namespace std::chrono_literals;

// this file contains the two nodelets that publish and subscribe in one file

class WheelPublisher:public rclcpp::Node{
  public:
    WheelPublisher():Node("wheel_publisher"){
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
    } 

  private:

    void timer_callback(){
      auto message = angular_vel::msg::DiffDriveOmega();
      
      // for now, set both to 0.5 rad/s
      message.set__wl(0.5);
      message.set__wr(0.5);

      publisher_->publish(message);
    }  

    rclcpp::Publisher<angular_vel::msg::DiffDriveOmega>::SharedPtr publisher_; // shared pointer to the nodelet
    rclcpp::TimerBase::SharedPtr timer_; // internal timer for publishing nodes
};


// TODO: create wheel subscriber that listens to angular velocity input



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelPublisher>());
  rclcpp::shutdown();
  return 0;
}
