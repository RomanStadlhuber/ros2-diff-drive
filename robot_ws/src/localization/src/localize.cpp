#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>
#include <math.h>
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1; // used to signify placeholder parameters

class Localize : public rclcpp::Node
{
public:
    Localize() : Node("localize")
    {
        subscriber_odometry_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "odometry",
            10,
            std::bind(&Localize::odometry_callback, this, _1));
    }

private:
    /**
         * the message contains the instantaneous directed distance
         * it then performs summing to integrate all instantaneous distances 
         */
    void odometry_callback(const geometry_msgs::msg::Twist::SharedPtr ds)
    {

        // caclculate new components of instantaneous distance
        // need to add the instantaneous angle difference to total angle to get actual world heading
        auto sx = ds->linear.x * cos(ds->angular.z + _position.angular.z);
        auto sy = ds->linear.x * sin(ds->angular.z + _position.angular.z);

        // update absolute distance
        _position.linear.x = _position.linear.x + sx;
        _position.linear.z = _position.linear.y + sy;

        //update absolute angle
        _position.angular.z = _position.angular.z + ds->angular.z;

        RCLCPP_INFO(
            this->get_logger(),
            "current position:\n\t%lf\n\t%lf\n\t%lf",
            _position.linear.x,
            _position.linear.y,
            _position.angular.z);
    }

    geometry_msgs::msg::Twist _position = geometry_msgs::msg::Twist();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_odometry_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localize>());
    rclcpp::shutdown();
    return 0;
}