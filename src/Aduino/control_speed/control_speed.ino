#include <micro_ros_arduino.h>
#include <SPI.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

// =====================
// Digipot + Motor Pins
// =====================
#define CS_DIGIPOT 7
#define OUT_PIN0 3
#define OUT_PIN1 4
#define OUT_PIN2 5

// =====================
// microROS
// =====================
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;
unsigned long last_ping_ms = 0;
const unsigned long ping_interval_ms = 500;

// =====================
// Digipot write
// =====================
void write_digipot(int val)
{
  if (val < 0) val = 0;
  if (val > 255) val = 255;

  digitalWrite(CS_DIGIPOT, LOW);
  SPI.transfer(0x11);
  SPI.transfer(val);
  digitalWrite(CS_DIGIPOT, HIGH);
}

// =====================
// Motor Control Logic
// =====================
void motor_forward()
{
  digitalWrite(OUT_PIN0, LOW);
  digitalWrite(OUT_PIN1, HIGH);
  delay(100);
  digitalWrite(OUT_PIN2, LOW);
  write_digipot(70);
}

void motor_reverse()
{
  digitalWrite(OUT_PIN0, HIGH);
  digitalWrite(OUT_PIN1, LOW);
  delay(100);
  digitalWrite(OUT_PIN2, LOW);
  write_digipot(70);
}

void motor_stop()
{
  digitalWrite(OUT_PIN0, HIGH);
  digitalWrite(OUT_PIN1, HIGH);
  digitalWrite(OUT_PIN2, HIGH);
  write_digipot(0);
}

// =====================
// ROS2 Callback
// =====================
void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist =
    (const geometry_msgs__msg__Twist *)msgin;

  float vx = twist->linear.x;

  if (vx > 0.01) {
    motor_forward();
  }
  else if (vx < -0.01) {
    motor_reverse();
  }
  else {
    motor_stop();
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  memset(&support, 0, sizeof(support));
  node = rcl_get_zero_initialized_node();
  subscriber = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();

  if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator)) {
    return false;
  }

  if (RCL_RET_OK != rclc_node_init_default(&node, "mkr_motor_node", "", &support)) {
    return false;
  }

  if (RCL_RET_OK != rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"))
  {
    return false;
  }

  if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 1, &allocator)) {
    return false;
  }

  if (RCL_RET_OK != rclc_executor_add_subscription(
      &executor,
      &subscriber,
      &msg,
      &cmd_vel_callback,
      ON_NEW_DATA))
  {
    return false;
  }

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  subscriber = rcl_get_zero_initialized_subscription();
  node = rcl_get_zero_initialized_node();
  memset(&support, 0, sizeof(support));
  executor = rclc_executor_get_zero_initialized_executor();
}

// =====================
// Setup
// =====================
void setup()
{
  set_microros_transports();   // USB transport

  SPI.begin();

  pinMode(CS_DIGIPOT, OUTPUT);
  pinMode(OUT_PIN0, OUTPUT);
  pinMode(OUT_PIN1, OUTPUT);
  pinMode(OUT_PIN2, OUTPUT);

  digitalWrite(CS_DIGIPOT, HIGH);

  motor_stop();

  delay(200);
}

// =====================
// Loop
// =====================
void loop()
{
  unsigned long now = millis();

  switch (agent_state) {
    case WAITING_AGENT:
      if (now - last_ping_ms >= ping_interval_ms) {
        last_ping_ms = now;
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          agent_state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (now - last_ping_ms >= ping_interval_ms) {
        last_ping_ms = now;
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          agent_state = AGENT_DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      motor_stop();
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
