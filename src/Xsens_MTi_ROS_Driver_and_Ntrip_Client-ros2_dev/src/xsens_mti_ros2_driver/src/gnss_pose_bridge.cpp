#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GnssPoseBridge : public rclcpp::Node
{
public:
  GnssPoseBridge()
  : Node("gnss_pose_bridge"), has_orientation_(false)
  {
    std::string fix_topic = declare_parameter<std::string>("fix_topic", "/fix");
    std::string orientation_topic =
      declare_parameter<std::string>("orientation_topic", "/filter/quaternion");
    std::string output_topic = declare_parameter<std::string>("output_topic", "/gnss_pose");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");
    int queue_size = declare_parameter<int>("queue_size", 10);
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, queue_size);

    orientation_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      orientation_topic,
      sensor_qos,
      std::bind(&GnssPoseBridge::orientationCallback, this, std::placeholders::_1));

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic,
      sensor_qos,
      std::bind(&GnssPoseBridge::fixCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %s from %s + %s",
      output_topic.c_str(),
      fix_topic.c_str(),
      orientation_topic.c_str());
  }

private:
  void orientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    latest_orientation_ = msg->quaternion;
    latest_orientation_frame_id_ = msg->header.frame_id;
    has_orientation_ = true;
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!has_orientation_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id =
      output_frame_id_.empty() ? msg->header.frame_id : output_frame_id_;

    // Match the existing Xsens GNSS pose convention: x=lat, y=lon, z=alt.
    pose_msg.pose.position.x = msg->latitude;
    pose_msg.pose.position.y = msg->longitude;
    pose_msg.pose.position.z = msg->altitude;
    pose_msg.pose.orientation = latest_orientation_;

    pose_pub_->publish(pose_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;

  geometry_msgs::msg::Quaternion latest_orientation_;
  std::string latest_orientation_frame_id_;
  std::string output_frame_id_;
  bool has_orientation_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
