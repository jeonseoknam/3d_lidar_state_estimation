#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class OdomToTwistConverter : public rclcpp::Node
{
public:
  OdomToTwistConverter()
  : Node("odom_to_twist_converter")
  {
    // Subscriber: /odom
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1,
      std::bind(&OdomToTwistConverter::odomCallback, this, std::placeholders::_1));

    // Publisher: /vehicle/twist_with_covariance
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/vehicle/twist_with_covariance", 1);

    RCLCPP_INFO(this->get_logger(), "Odom -> TwistWithCovariance converter node started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out_msg;
    out_msg.header = msg->header;
    out_msg.twist = msg->twist;  // Odometry -> use twist

    pub_twist_->publish(out_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTwistConverter>());
  rclcpp::shutdown();
  return 0;
}

