#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_vel_ned.hpp>

class GnssOdometryBridge : public rclcpp::Node
{
public:
  GnssOdometryBridge()
  : Node("gnss_odometry_bridge")
  {
    std::string fix_topic = declare_parameter<std::string>("fix_topic", "/fix");
    std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    std::string velocity_topic =
      declare_parameter<std::string>("velocity_topic", "/ubx_nav_vel_ned");
    std::string output_topic = declare_parameter<std::string>("output_topic", "/gnss_odometry");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    int queue_size = declare_parameter<int>("queue_size", 10);
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, queue_size);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      sensor_qos,
      std::bind(&GnssOdometryBridge::imuCallback, this, std::placeholders::_1));

    velocity_sub_ = create_subscription<ublox_ubx_msgs::msg::UBXNavVelNED>(
      velocity_topic,
      sensor_qos,
      std::bind(&GnssOdometryBridge::velocityCallback, this, std::placeholders::_1));

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic,
      sensor_qos,
      std::bind(&GnssOdometryBridge::fixCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %s from %s + %s",
      output_topic.c_str(),
      fix_topic.c_str(),
      imu_topic.c_str());
  }

private:
  struct UTMCoordinate
  {
    double easting = 0.0;
    double northing = 0.0;
    double altitude = 0.0;
    int zone = 0;
  };

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
    has_imu_ = true;
  }

  void velocityCallback(const ublox_ubx_msgs::msg::UBXNavVelNED::SharedPtr msg)
  {
    latest_velocity_ = *msg;
    has_velocity_ = true;
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!has_imu_) {
      return;
    }

    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      return;
    }

    if (utm0_.zone == 0) {
      initUTM(msg->latitude, msg->longitude, msg->altitude);
    }

    double utm_easting = 0.0;
    double utm_northing = 0.0;
    LLtoUTM(msg->latitude, msg->longitude, utm0_.zone, utm_easting, utm_northing);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    odom_msg.pose.pose.position.x = utm_easting - utm0_.easting;
    odom_msg.pose.pose.position.y = utm_northing - utm0_.northing;
    odom_msg.pose.pose.position.z = msg->altitude - utm0_.altitude;
    odom_msg.pose.pose.orientation = latest_imu_.orientation;

    // Copy GNSS position covariance into the pose covariance matrix.
    odom_msg.pose.covariance[0] = msg->position_covariance[0];
    odom_msg.pose.covariance[1] = msg->position_covariance[1];
    odom_msg.pose.covariance[2] = msg->position_covariance[2];
    odom_msg.pose.covariance[6] = msg->position_covariance[3];
    odom_msg.pose.covariance[7] = msg->position_covariance[4];
    odom_msg.pose.covariance[8] = msg->position_covariance[5];
    odom_msg.pose.covariance[12] = msg->position_covariance[6];
    odom_msg.pose.covariance[13] = msg->position_covariance[7];
    odom_msg.pose.covariance[14] = msg->position_covariance[8];
    odom_msg.pose.covariance[21] = latest_imu_.orientation_covariance[0];
    odom_msg.pose.covariance[28] = latest_imu_.orientation_covariance[4];
    odom_msg.pose.covariance[35] = latest_imu_.orientation_covariance[8];

    if (has_velocity_) {
      // Convert u-blox NED velocity (cm/s) into ENU linear velocity (m/s)
      // to match the UTM-based odometry position convention used here.
      odom_msg.twist.twist.linear.x = static_cast<double>(latest_velocity_.vel_e) * 0.01;
      odom_msg.twist.twist.linear.y = static_cast<double>(latest_velocity_.vel_n) * 0.01;
      odom_msg.twist.twist.linear.z = -static_cast<double>(latest_velocity_.vel_d) * 0.01;

      double speed_variance = std::pow(static_cast<double>(latest_velocity_.s_acc) * 0.01, 2);
      odom_msg.twist.covariance[0] = speed_variance;
      odom_msg.twist.covariance[7] = speed_variance;
      odom_msg.twist.covariance[14] = speed_variance;
    }

    odom_msg.twist.twist.angular = latest_imu_.angular_velocity;
    odom_msg.twist.covariance[21] = latest_imu_.angular_velocity_covariance[0];
    odom_msg.twist.covariance[28] = latest_imu_.angular_velocity_covariance[4];
    odom_msg.twist.covariance[35] = latest_imu_.angular_velocity_covariance[8];

    odom_pub_->publish(odom_msg);
  }

  void initUTM(double latitude, double longitude, double altitude)
  {
    int zone_number = calcZone(latitude, longitude);
    utm0_.zone = zone_number;
    utm0_.altitude = altitude;
    LLtoUTM(latitude, longitude, utm0_.zone, utm0_.easting, utm0_.northing);
    RCLCPP_INFO(
      get_logger(),
      "Initialized UTM zone %d at easting %.3f northing %.3f altitude %.3f",
      utm0_.zone,
      utm0_.easting,
      utm0_.northing,
      utm0_.altitude);
  }

  int calcZone(double latitude, double longitude) const
  {
    double normalized_longitude =
      (longitude + 180.0) - static_cast<int>((longitude + 180.0) / 360.0) * 360.0 - 180.0;

    int zone_number = static_cast<int>((normalized_longitude + 180.0) / 6.0) + 1;

    if (latitude >= 56.0 && latitude < 64.0 && normalized_longitude >= 3.0 &&
      normalized_longitude < 12.0)
    {
      zone_number = 32;
    }

    if (latitude >= 72.0 && latitude < 84.0) {
      if (normalized_longitude >= 0.0 && normalized_longitude < 9.0) {
        zone_number = 31;
      } else if (normalized_longitude >= 9.0 && normalized_longitude < 21.0) {
        zone_number = 33;
      } else if (normalized_longitude >= 21.0 && normalized_longitude < 33.0) {
        zone_number = 35;
      } else if (normalized_longitude >= 33.0 && normalized_longitude < 42.0) {
        zone_number = 37;
      }
    }

    return zone_number;
  }

  void LLtoUTM(
    double latitude,
    double longitude,
    int zone_number,
    double & utm_easting,
    double & utm_northing) const
  {
    const double a = 6378137.0;
    const double ecc_squared = 0.00669438;
    const double k0 = 0.9996;

    double normalized_longitude =
      (longitude + 180.0) - static_cast<int>((longitude + 180.0) / 360.0) * 360.0 - 180.0;

    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = normalized_longitude * M_PI / 180.0;
    double lon_origin = (zone_number - 1) * 6.0 - 180.0 + 3.0;
    double lon_origin_rad = lon_origin * M_PI / 180.0;

    double ecc_prime_squared = ecc_squared / (1.0 - ecc_squared);
    double n = a / std::sqrt(1.0 - ecc_squared * std::sin(lat_rad) * std::sin(lat_rad));
    double t = std::tan(lat_rad) * std::tan(lat_rad);
    double c = ecc_prime_squared * std::cos(lat_rad) * std::cos(lat_rad);
    double a_term = std::cos(lat_rad) * (lon_rad - lon_origin_rad);

    double m = a * (
      (1.0 - ecc_squared / 4.0 - 3.0 * ecc_squared * ecc_squared / 64.0 -
      5.0 * std::pow(ecc_squared, 3) / 256.0) * lat_rad -
      (3.0 * ecc_squared / 8.0 + 3.0 * ecc_squared * ecc_squared / 32.0 +
      45.0 * std::pow(ecc_squared, 3) / 1024.0) * std::sin(2.0 * lat_rad) +
      (15.0 * ecc_squared * ecc_squared / 256.0 +
      45.0 * std::pow(ecc_squared, 3) / 1024.0) * std::sin(4.0 * lat_rad) -
      (35.0 * std::pow(ecc_squared, 3) / 3072.0) * std::sin(6.0 * lat_rad));

    utm_easting = k0 * n * (
      a_term + (1.0 - t + c) * std::pow(a_term, 3) / 6.0 +
      (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ecc_prime_squared) *
      std::pow(a_term, 5) / 120.0) + 500000.0;

    utm_northing = k0 * (
      m + n * std::tan(lat_rad) * (
        a_term * a_term / 2.0 +
        (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a_term, 4) / 24.0 +
        (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ecc_prime_squared) *
        std::pow(a_term, 6) / 720.0));

    if (latitude < 0.0) {
      utm_northing += 10000000.0;
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavVelNED>::SharedPtr velocity_sub_;

  sensor_msgs::msg::Imu latest_imu_;
  ublox_ubx_msgs::msg::UBXNavVelNED latest_velocity_;
  UTMCoordinate utm0_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool has_imu_ = false;
  bool has_velocity_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssOdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
