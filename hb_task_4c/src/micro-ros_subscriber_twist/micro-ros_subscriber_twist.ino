#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define LED_PIN 13
Servo servo_top;
Servo servo_right;
Servo servo_left;
Servo servo_arm;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  //digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
}

void setup() {
  set_microros_wifi_transports("A.T.O.M_Labs", "atom281121", "192.168.0.18", 8888);
  Serial.begin(9600);
  servo_top.attach(27);
  servo_right.attach(26);
  servo_left.attach(25);
  servo_arm.attach(33);
  servo_top.write(90);
  servo_right.write(90);
  servo_left.write(90);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/bot3"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  Serial.print("top-velocity: ");
  servo_top.write(msg.linear.x);
  Serial.println(msg.linear.x); 

  Serial.print("right-velocity: ");
  servo_right.write(msg.linear.y);
  Serial.println(msg.linear.y);

  Serial.print("left-velocity: ");
  servo_left.write(msg.linear.z);
  Serial.println(msg.linear.z);

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
