#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

class LidarToInitialPose : public rclcpp::Node {
public:
  LidarToInitialPose() : Node("lidar_to_initialpose") {
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/current_pose", 10,
      std::bind(&LidarToInitialPose::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

    // // EKF 서비스 클라이언트 생성
    // client_ = this->create_client<std_srvs::srv::SetBool>("/trigger_node");

    published_ = false;
  }

private:
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (published_) return;  // 첫 메시지만 사용

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header = msg->header;   // frame_id = "map"
    init_pose.pose.pose = msg->pose;

    // covariance 값 세팅
    for (int i = 0; i < 36; i++) init_pose.pose.covariance[i] = 0.001;
    init_pose.pose.covariance[0] = 0.25;   // x
    init_pose.pose.covariance[7] = 0.25;   // y
    init_pose.pose.covariance[35] = 0.068; // yaw

    pub_->publish(init_pose);
    RCLCPP_INFO(this->get_logger(), "Published first /current_pose as /initialpose");

    published_ = true;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  bool published_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToInitialPose>());
  rclcpp::shutdown();
  return 0;
}
