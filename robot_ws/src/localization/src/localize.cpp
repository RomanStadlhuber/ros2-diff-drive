#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>
#include <math.h>
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1; // used to signify placeholder parameters


class Localize:public rclcpp::Node{
    public:
        Localize() :Node("localize"){

            // set initial position
            _position.x = _position.y = _position.z = 0;

            subscriber_odometry_ = this -> create_subscription<geometry_msgs::msg::Vector3>(
                "odometry",
                10,
                std::bind(&Localize::odometry_callback, this, _1)
            );
        }
    private:

        /**
         * the message contains the instantaneous directed distance
         * it then performs summing to integrate all instantaneous distances 
         */
        void odometry_callback(const geometry_msgs::msg::Vector3::SharedPtr ds){
            _position.x = _position.x + ds -> x;
            _position.y = _position.y + ds -> y;

            RCLCPP_INFO(this -> get_logger(), "current position:\n\t%lf\n\t%lf\n\t%lf", _position.x, _position.y, _position.z);
        }

        geometry_msgs::msg::Vector3 _position = geometry_msgs::msg::Vector3();

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_odometry_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localize>());
  rclcpp::shutdown();
  return 0;
}
 