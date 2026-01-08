#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class PoseConverter : public rclcpp::Node
{
public:
  PoseConverter() : Node("pose_to_pose_with_cov")
  {
    // 파라미터 선언 (실행 중 동적 조정 가능)
    declare_parameter<std::string>("output_topic", "/ndt_pose_with_covariance");
    declare_parameter<std::string>("expected_frame_id", "map");   // ekf_localizer의 pose_frame_id와 동일해야 함
    declare_parameter<double>("sigma_xy_m", 0.01);                // 3 cm
    declare_parameter<double>("sigma_yaw_deg", 0.3);             // 0.5 degree
    declare_parameter<bool>("use_zrp", false);                   // Z/roll/pitch를 관측에 포함할지
    declare_parameter<double>("sigma_z_m", 0.05);                // 사용할 경우 5 cm
    declare_parameter<double>("sigma_rp_deg", 1.0);              // 사용할 경우 1 degree
    declare_parameter<double>("big_unc", 1e6);                   // 관측하지 않는 축에 줄 큰 분산

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ndt_pose", rclcpp::QoS(1),
      std::bind(&PoseConverter::callback, this, std::placeholders::_1));

    std::string out_topic = get_parameter("output_topic").as_string();
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(out_topic, rclcpp::QoS(1));
  }

private:
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const std::string expected_frame = get_parameter("expected_frame_id").as_string();
    const double sigma_xy_m     = get_parameter("sigma_xy_m").as_double();
    const double sigma_yaw_deg  = get_parameter("sigma_yaw_deg").as_double();
    const bool   use_zrp        = get_parameter("use_zrp").as_bool();
    const double sigma_z_m      = get_parameter("sigma_z_m").as_double();
    const double sigma_rp_deg   = get_parameter("sigma_rp_deg").as_double();
    const double BIG            = get_parameter("big_unc").as_double();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    // EKF의 params.pose_frame_id와 반드시 같아야 한다. (기본: "map")
    out.header.frame_id = expected_frame;
    out.pose.pose = msg->pose;

    // 6x6 공분산 0으로 초기화(상관항 제거)
    for (double &v : out.pose.covariance) v = 0.0;

    // index: 0:x, 7:y, 14:z, 21:roll, 28:pitch, 35:yaw
    const double var_xy   = sigma_xy_m * sigma_xy_m;                       // (예) 0.03m → 9e-4
    const double var_yaw  = std::pow(sigma_yaw_deg * M_PI / 180.0, 2.0);  // (예) 0.5° → ~7.6e-5

    out.pose.covariance[0]  = var_xy;   // var(x)
    out.pose.covariance[7]  = var_xy;   // var(y)
    out.pose.covariance[35] = var_yaw;  // var(yaw)

    if (use_zrp) {
      const double var_z   = sigma_z_m * sigma_z_m;
      const double var_rp  = std::pow(sigma_rp_deg * M_PI / 180.0, 2.0);
      out.pose.covariance[14] = var_z;   // var(z)
      out.pose.covariance[21] = var_rp;  // var(roll)
      out.pose.covariance[28] = var_rp;  // var(pitch)
    } else {
      // 관측하지 않는 축은 매우 큰 분산으로 "무시" 표기
      out.pose.covariance[14] = BIG; // z
      out.pose.covariance[21] = BIG; // roll
      out.pose.covariance[28] = BIG; // pitch
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConverter>());
  rclcpp::shutdown();
  return 0;
}
