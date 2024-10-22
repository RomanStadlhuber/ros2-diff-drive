#include <cstdio>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <math.h>
#include "angular_vel/msg/diff_drive_omega.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * custom definitions for parameter fields
 */

#define PARAM_AXISLEN "axislen"
#define PARAM_WHEELRADIUS "wheelradius"

using namespace std::chrono_literals;
using std::placeholders::_1; // used to signify placeholder parameters

class Odometry : public rclcpp::Node
{
public:
  Odometry() : Node("odometry")
  {

    // declare and read parameters

    this->declare_parameters(_namespace, _arglist);

    this->get_parameter(PARAM_AXISLEN, _axislen);
    _axislen = _axislen * 0.001;

    this -> get_parameter(PARAM_WHEELRADIUS, _wheelRadius);
    _wheelRadius = _wheelRadius * 0.001;

    RCLCPP_INFO(this->get_logger(), "init node odometry with\n\taxislen: %lf\n\twheelradius: %lf", _axislen, _wheelRadius);

    // bind subscription receiver and odometry publisher callbacks

    subscription_ = this->create_subscription<angular_vel::msg::DiffDriveOmega>(
        "angular_vel_out",
        10,
        // the last argument indicated the number of paremeters the bound function needs
        std::bind(&Odometry::sub_callback, this, _1));

    publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    publisher_odometry_ = this -> create_publisher<geometry_msgs::msg::Twist>("odometry", 10);

  }

private:
  void sub_callback(const angular_vel::msg::DiffDriveOmega::SharedPtr omega)
  {

    // log received information
    RCLCPP_INFO(this->get_logger(), "w-left: %lf, w-right: %lf", omega->wl, omega->wr);

    // calculate planar and angular velocity
    _dv = calc_vel_planar(&omega->wl, &omega->wr, &_wheelRadius);
    _dw = calc_vel_angular(&omega->wl, &omega->wr, &_axislen, &_wheelRadius);

    // log calculation results
    RCLCPP_INFO(this->get_logger(), "dv: %lf, dw: %lf", _dv, _dw);

    // ----------------- prepare twist message ------------------

    auto planar = geometry_msgs::msg::Vector3();

    planar.x = _dv; // the simulation only receives frontal velocity
    planar.y = planar.z = 0.0;

    auto angular = geometry_msgs::msg::Vector3();

    angular.x = angular.y = 0.0;

    angular.z = _dw;

    auto message = geometry_msgs::msg::Twist();

    message.angular = angular;
    message.linear = planar;

    /**
       * TODO: vector3 message definiton for directed distance
       * 1 - create stopwatch to measure time between incoming subscription messages
       * 2 - create directed instantaneous velocity vector with x = dv * cos(dw) and y = dv * sin dw
       * 3 - multiply vel with dt (stopwatch) to receive instantaneous distance
       * 4 - publish instantaneous distance to "/odometry"
       */

    //----------------- publish message -------------------------

    publisher_twist_->publish(message);

    //-- calculate and publish instantaneous directed distance --

    rclcpp::Time tnow = nodeclock->now();

    rclcpp::Duration dt = tnow - _lastmsg;

    _lastmsg = tnow;

    RCLCPP_INFO(this->get_logger(), "dt: %lf[s]", dt.seconds());

    planar = geometry_msgs::msg::Vector3();

    planar.x = _dv * dt.seconds(); // frontal distance in a timeframe
    planar.y = planar.z = 0.0;

    angular = geometry_msgs::msg::Vector3();

    angular.x = angular.y = 0.0;

    angular.z = _dw * dt.seconds(); // difference in angle in a timeframe

    message = geometry_msgs::msg::Twist();

    message.angular = angular;
    message.linear = planar;

    publisher_odometry_->publish(message);

// ------------------------------------------------------------
  }

  // local variabled and functions used to calculate odometry

  double calc_vel_angular(double *wl, double *wr, double *axislen, double *radius)
  {
    return (((*wr) * (*radius)) - ((*wl) * (*radius))) / (*axislen);
  }

  double calc_vel_planar(double *wl, double *wr, double *radius)
  {
    return (((*wr) * (*radius)) + ((*wl) * (*radius))) / 2;
  }

  double _axislen = 0.12;         // unit is meters
  double _wheelRadius = 0.004075; // unit is meters

  /**
     * define odometry buffer values
     */

  double _dv = 0.0; // instantaneous planar velocity "delta v"
  double _dw = 0.0; // instantaneous angular velocity "delta w"

  

  // ROS specific variables for nodes, timing and parameters

  rclcpp::Subscription<angular_vel::msg::DiffDriveOmega>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_odometry_;
  /**
   * NOTE: see https://answers.ros.org/question/321436/ros2-msg-stamp-time-difference/
   */
  rclcpp::Clock::SharedPtr nodeclock = this->get_clock();

  rclcpp::Time _lastmsg = nodeclock->now();

  // declare paramters for the robot nodes here
  const std::string _namespace = "";

  const std::map<std::string, double> _arglist = {
      {PARAM_AXISLEN, 0.0},    // the length of the axis i.e. the direct distance between contact center points
      {PARAM_WHEELRADIUS, 0.0}, // radius of wheels
  };

  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
