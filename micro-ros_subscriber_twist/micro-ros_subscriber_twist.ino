#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 13

rcl_subscription_t subscriber;
rcl_subscription_t subscriber_bool;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Bool topic;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

Servo servo_top;
Servo servo_right;
Servo servo_left;
Servo servo_arm;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // Update servo positions based on twist message data
  servo_top.write(msg->linear.x);
  servo_right.write(msg->linear.y);
  servo_left.write(msg->linear.z);
}

//bool message callback
void subscription_callback_bool(const void *msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  // Update servo_arm position based on boolean message data
  if (msg->data) {
    servo_arm.write(30); // Pen DOWN
  } else {
    servo_arm.write(5); // Pen UP
  }
}

void setup() {
  set_microros_wifi_transports("shlok", "12345678","192.168.65.128", 8888);
  Serial.begin(9600);

  servo_top.attach(27);
  servo_right.attach(26);
  servo_left.attach(25);
  servo_arm.attach(33);

  servo_arm.write(5); // Initialize servo positions
  servo_top.write(90);
  servo_right.write(90);
  servo_left.write(90);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_3", "", &support));

  // Initialize subscribers
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel/bot3"));

  RCCHECK(rclc_subscription_init_default(
            &subscriber_bool,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/pen3_down"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Add subscriptions to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_bool, &topic, &subscription_callback_bool, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
