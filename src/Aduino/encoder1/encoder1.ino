#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/timer.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
#include <string.h>

// 🔹 ตั้งค่าขาสัญญาณจาก Encoder
#define ENCODER_PIN A3

// 🔹 ตัวแปรสำหรับ Encoder
volatile int pulseCount = 0;
unsigned long lastPulseTime = 0;
const int checkInterval = 500;   // ms
const int debounceTime = 5;      // ms

const int pulsePerRevolution = 60;   // pulses / wheel revolution
const float wheelCircumference = 1.57; // meters

// 🔹 micro-ROS
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;
unsigned long last_ping_ms = 0;
const unsigned long ping_interval_ms = 500;

// ใช้เก็บเวลา integration
static unsigned long last_time = 0;

// 📌 Timer callback
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  if (timer == NULL) return;

  unsigned long currentTime = millis();

  // ถ้าไม่มี pulse นาน แสดงว่ารถหยุด
  if (currentTime - lastPulseTime > checkInterval) {
    pulseCount = 0;
  }

  // คำนวณความเร็ว
  float RPM = (float)pulseCount / pulsePerRevolution * (60.0 / 0.5);
  float velocity_mps = (RPM / 60.0) * wheelCircumference;

  // ===== เวลา (ROS time) =====
  int64_t now = rmw_uros_epoch_millis();
  odom_msg.header.stamp.sec = now / 1000;
  odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;

  // ===== frame =====
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom_wheel");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  // ===== คำนวณ dt จริง =====
  if (last_time == 0) last_time = currentTime;
  float dt = (currentTime - last_time) / 1000.0;
  last_time = currentTime;

  // ===== integrate position =====
  odom_msg.pose.pose.position.x += velocity_mps * dt;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;

  // ===== velocity =====
  odom_msg.twist.twist.linear.x = velocity_mps;
  odom_msg.twist.twist.angular.z = 0.0;

  // ===== orientation (ให้ IMU/EKF จัดการ yaw) =====
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;

  // ===== covariance (ไม่เป็นศูนย์) =====
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i]  = (i % 7 == 0) ? 0.1 : 0.0;
    odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
  }

  // publish
  rcl_publish(&odom_publisher, &odom_msg, NULL);

  pulseCount = 0;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  memset(&support, 0, sizeof(support));
  node = rcl_get_zero_initialized_node();
  odom_publisher = rcl_get_zero_initialized_publisher();
  timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();

  if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator)) {
    return false;
  }

  if (RCL_RET_OK != rclc_node_init_default(&node, "wheel_encoder_node", "", &support)) {
    return false;
  }

  if (RCL_RET_OK != rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/wheel"))
  {
    return false;
  }

  if (RCL_RET_OK != rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(checkInterval),
      timer_callback))
  {
    return false;
  }

  if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 1, &allocator)) {
    return false;
  }

  if (RCL_RET_OK != rclc_executor_add_timer(&executor, &timer)) {
    return false;
  }

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_timer_fini(&timer);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  node = rcl_get_zero_initialized_node();
  odom_publisher = rcl_get_zero_initialized_publisher();
  timer = rcl_get_zero_initialized_timer();
  memset(&support, 0, sizeof(support));
  executor = rclc_executor_get_zero_initialized_executor();
}

void setup()
{
  Serial.begin(115200);
  unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start < 2000)) {
    delay(10);
  }

  pinMode(ENCODER_PIN, INPUT_PULLUP);

  set_microros_transports();
}

void loop()
{
  static int lastState = HIGH;
  int currentState = digitalRead(ENCODER_PIN);
  unsigned long currentTime = millis();

  // ตรวจจับ pulse (falling edge)
  if (lastState == HIGH &&
      currentState == LOW &&
      (currentTime - lastPulseTime > debounceTime))
  {
    pulseCount++;
    lastPulseTime = currentTime;
  }

  lastState = currentState;

  switch (agent_state) {
    case WAITING_AGENT:
      if (currentTime - last_ping_ms >= ping_interval_ms) {
        last_ping_ms = currentTime;
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          agent_state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        last_time = 0;
        agent_state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (currentTime - last_ping_ms >= ping_interval_ms) {
        last_ping_ms = currentTime;
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          agent_state = AGENT_DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
