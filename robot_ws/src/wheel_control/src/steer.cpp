#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <math.h>
#include "geometry_msgs/msg/twist.hpp"

#define TOPIC_ANGULAR_VEL_IN "angular_vel_in"
#define TOPIC_POSE_OUT "pose"
#define TOPIC_POSE_IN "pose_in"

#define PARAM_MAXANGULARVEL "maxangularvel"
#define PARAM_AXISLEN "axislen"
#define PARAM_WHEELRADIUS "wheelradius"

using std::placeholders::_1;

class Steer : public rclcpp::Node
{

public:
    Steer() : Node("localize")
    {
        // declare and read parameters
        this->declare_parameters(_namespace, _arglist);

        this->get_parameter(PARAM_AXISLEN, _axislen);
        _axislen = _axislen * 0.001;

        this->get_parameter(PARAM_WHEELRADIUS, _wheelradius);
        _wheelradius = _wheelradius * 0.001;

        this->get_parameter(PARAM_MAXANGULARVEL, _maxangularvel); // unit is rad, no need to convert

        // create target position subscriber
        subscriber_target_pose_ = this->create_subscription<geometry_msgs::msg::Twist>(
            TOPIC_POSE_IN,
            10,
            std::bind(&Steer::target_pose_callback, this, _1));

        // create current position subscriber
        subscriber_current_pose_ = this->create_subscription<geometry_msgs::msg::Twist>(
            TOPIC_POSE_IN,
            10,
            std::bind(&Steer::current_pose_callback, this, _1));
    }

private:
    void target_pose_callback(const geometry_msgs::msg::Twist::SharedPtr target_pose)
    {
        _tpose = *target_pose;
    }

    void current_pose_callback(const geometry_msgs::msg::Twist::SharedPtr curr_pose)
    {
        /**
         * TODO: 
         *  - calculate difference between target pose and current pose
         *  - calculate required left and right wheel angular velocities
         *  - publish angular velocity to "angular_vel_in"
         */

        /**
         * calclate delta distance vector based on current and target pose linear vector
         */
        _dpose.linear = [](auto curr, auto target) {
            auto diff = geometry_msgs::msg::Vector3();
            diff.x = target.x - curr.x;
            diff.y = target.y - curr.y;
            diff.z = 0;
            return diff;
        }(curr_pose->linear, _tpose.linear);

        _dpose.angular.z = atan2(_dpose.linear.x, _dpose.linear.y);

        // calculate wheen angular velocities
        std::tuple<double, double> angularVelocities = [](double r, double b, double wmax, double dphi) {
            if (dphi == 0.0)
                return std::tuple<double, double>(wmax, wmax);
            else if (dphi > 0.0)
                return std::tuple<double, double>(wmax, -wmax);
            else // dphi < 0.0
                return std::tuple<double, double>(-wmax, wmax);
        }(_wheelradius, _axislen, _maxangularvel, _dpose.angular.z);

        /**
         * NOTE: when delta distance is within a margin, stop moving the weels
         * TODO: publish to angulat_vel_in topic to steer the robot
         */
    }

    geometry_msgs::msg::Twist _tpose = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist _dpose = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist _curr_pose = geometry_msgs::msg::Twist();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_target_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_current_pose_;

    // declare paramters for the robot nodes here
    const std::string _namespace = "";

    const std::map<std::string, double> _arglist = {
        {PARAM_AXISLEN, 0.0},     // the length of the axis i.e. the direct distance between contact center points
        {PARAM_WHEELRADIUS, 0.0}, // radius of wheels
        {PARAM_MAXANGULARVEL, 0.0},
    };

    double _axislen, _wheelradius, _maxangularvel = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Steer>());
    rclcpp::shutdown();
    return 0;
}