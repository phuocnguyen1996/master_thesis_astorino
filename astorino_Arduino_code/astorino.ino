// Include required packages and libraries

// ROS packages
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

// Arduino libraries, such as steppers, I/O Board SX1509 for controlling gripper
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>           // Include the I2C library (required)
#include <SparkFunSX1509.h> 

// Define the connection pins for motors and others
#define EN_PIN_M1 33
#define EN_PIN_M2 37
#define EN_PIN_M3 38
#define EN_PIN_M456 36
#define LED_PIN 13

#define STEP_PIN_M1 30
#define STEP_PIN_M2 29
#define STEP_PIN_M3 28
#define STEP_PIN_M4 27
#define STEP_PIN_M5 26
#define STEP_PIN_M6 25

#define DIR_PIN_M1 2
#define DIR_PIN_M2 10
#define DIR_PIN_M3 9
#define DIR_PIN_M4 32
#define DIR_PIN_M5 31
#define DIR_PIN_M6 6

// Some setup for using microROS, read https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/
// and some examples for more information
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Constant ratios between joints' angle and motors' pulse 
const float JOINT1_RATIO = 600; // For example, 600 pulses at motor 1 will rotate joint 1 degree 
const float JOINT2_RATIO = 630;
const float JOINT3_RATIO = -443;
const float JOINT4_RATIO = -354;
const float JOINT5_RATIO = 370;
const float JOINT6_RATIO = -142;

// Some publisher and subscriber setup for microROS
rcl_subscription_t subscriber; // Enable motor
rcl_subscription_t subscriber_move; // Move motor
rcl_subscription_t subscriber_gripper; // Open/close gripper
std_msgs__msg__Bool msg;
std_msgs__msg__Bool msg_finished_action;
std_msgs__msg__Float64MultiArray msg_move;
rclc_executor_t executor;
rclc_executor_t executor_move;
rclc_executor_t executor_gripper;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher_finished_action; // For publishing a signal whenever an operation is executed
rclc_executor_t executor_finished_action;


long pos[6]; // Target position for 6 joints

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address
SX1509 io;                        // Create an SX1509 object to be used throughout

// SX1509 and gripper pin definitions:
const byte SX1509_LED_PIN = 12; 
const byte OPEN_PIN = 13;
const byte CLOSE_PIN = 15;


AccelStepper stepper1(1, STEP_PIN_M1, DIR_PIN_M1);
AccelStepper stepper2(1, STEP_PIN_M2, DIR_PIN_M2);
AccelStepper stepper3(1, STEP_PIN_M3, DIR_PIN_M3);
AccelStepper stepper4(1, STEP_PIN_M4, DIR_PIN_M4);
AccelStepper stepper5(1, STEP_PIN_M5, DIR_PIN_M5);
AccelStepper stepper6(1, STEP_PIN_M6, DIR_PIN_M6);

MultiStepper multi_steppers;

// Some publisher and subscriber setup for microROS
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void callback(const void * msgin) // Enable motors
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data == true) // High for enabling
  {
    digitalWrite(EN_PIN_M1, HIGH);
    digitalWrite(EN_PIN_M2, HIGH);
    digitalWrite(EN_PIN_M3, HIGH);
    digitalWrite(EN_PIN_M456, HIGH);
  }
    
  else // Low for disabling
  {
    digitalWrite(EN_PIN_M1, LOW);
    digitalWrite(EN_PIN_M2, LOW);
    digitalWrite(EN_PIN_M3, LOW);
    digitalWrite(EN_PIN_M456, LOW);
  }
    
}

void callback_gripper(const void * msgin) // Open/Close gripper
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  std_msgs__msg__Bool msg_finished_action;

  if (msg->data == true) //open
  {
    io.digitalWrite(CLOSE_PIN, LOW);
    io.digitalWrite(OPEN_PIN, HIGH);
    delay(2300);  // Set delay time for keeping the values at the pins for an amount of time 
    io.digitalWrite(CLOSE_PIN, LOW);
    io.digitalWrite(OPEN_PIN, LOW);
    delay(1000);
  }
    
  else //close
  {
    io.digitalWrite(CLOSE_PIN, HIGH);
    io.digitalWrite(OPEN_PIN, LOW);
    delay(2300);
    io.digitalWrite(CLOSE_PIN, LOW);
    io.digitalWrite(OPEN_PIN, LOW);
    delay(1000);
  }

  // Publishing a signal when finish
  msg_finished_action.data = true;
  RCSOFTCHECK(rcl_publish(&publisher_finished_action, &msg_finished_action, NULL));
}

void callback_move(const void * msgin) //Rotate motors
{
  const std_msgs__msg__Float64MultiArray * msg_move = (const std_msgs__msg__Float64MultiArray *)msgin;

  std_msgs__msg__Bool msg_action;

  pos[0] = int(msg_move->data.data[0]*JOINT1_RATIO);
  pos[1] = int(msg_move->data.data[1]*JOINT2_RATIO);
  pos[2] = int(msg_move->data.data[2]*JOINT3_RATIO);
  pos[3] = int(msg_move->data.data[3]*JOINT4_RATIO);
  pos[4] = int(msg_move->data.data[4]*JOINT5_RATIO);
  pos[5] = int(msg_move->data.data[5]*JOINT6_RATIO);

  multi_steppers.moveTo(pos);
  multi_steppers.runSpeedToPosition();

  // Publishing a signal when finish
  msg_action.data = true;
  RCSOFTCHECK(rcl_publish(&publisher_finished_action, &msg_action, NULL));

  
}

// Arduino setting up
void setup() {

  set_microros_transports();

  Wire.begin();


  if (io.begin(SX1509_ADDRESS) == false)
  {
    // Serial.println("Failed to communicate. Check wiring and address of SX1509.");
    while (1)
      ; // If we fail to communicate, loop forever.
  }

  // Pin working modes
  io.pinMode(SX1509_LED_PIN, OUTPUT);
  io.pinMode(OPEN_PIN, OUTPUT);
  io.pinMode(CLOSE_PIN, OUTPUT);
  io.digitalWrite(SX1509_LED_PIN, HIGH);
  io.digitalWrite(OPEN_PIN, LOW);
  io.digitalWrite(CLOSE_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  pinMode(EN_PIN_M456, OUTPUT);
  pinMode(EN_PIN_M1, OUTPUT);
  pinMode(EN_PIN_M2, OUTPUT);
  pinMode(EN_PIN_M3, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(EN_PIN_M456, LOW);
  digitalWrite(EN_PIN_M1, LOW);
  digitalWrite(EN_PIN_M2, LOW);
  digitalWrite(EN_PIN_M3, LOW);

  // Setup steppers parameters
  stepper1.setMaxSpeed(2000*2);
  stepper2.setMaxSpeed(2000*2);
  stepper3.setMaxSpeed(2000*2);
  stepper4.setMaxSpeed(2000*2);
  stepper5.setMaxSpeed(2000*2);
  stepper6.setMaxSpeed(2000*2);

  stepper1.setAcceleration(1000*2);
  stepper2.setAcceleration(1000*2);
  stepper3.setAcceleration(1000*2);
  stepper4.setAcceleration(1000*2);
  stepper5.setAcceleration(1000*2);
  stepper6.setAcceleration(1000*2);

  multi_steppers.addStepper(stepper1);
  multi_steppers.addStepper(stepper2);
  multi_steppers.addStepper(stepper3);
  multi_steppers.addStepper(stepper4);
  multi_steppers.addStepper(stepper5);
  multi_steppers.addStepper(stepper6);

  delay(1000);

  // MicroROS setup, view the webpage for more information
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "enable_motors"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_move,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "rotate_motors"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_gripper,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "open_gripper"));    

  RCCHECK(rclc_publisher_init_default(
    &publisher_finished_action,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "response"));


  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_move, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_gripper, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_finished_action, &support.context, 1, &allocator));


// Because I use MultyArray msg, so it is necessary to handle memory,
// which is different from other msg types. More information viewing in the link below
// https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
  static double_t memory[6];
  msg_move.data.data = memory;
  msg_move.data.capacity = 6;
  msg_move.data.size = 0;

  msg_move.layout.dim.capacity = 6;
  msg_move.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_move.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  msg_move.layout.dim.size = 0;

  for(size_t i = 0; i < msg_move.layout.dim.capacity; i++){
    msg_move.layout.dim.data[i].label.capacity = 20;
    msg_move.layout.dim.data[i].label.size = 0;
    msg_move.layout.dim.data[i].label.data = (char*) malloc(msg_move.layout.dim.data[i].label.capacity * sizeof(char));
  }

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_move, &subscriber_move, &msg_move, &callback_move, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_gripper, &subscriber_gripper, &msg, &callback_gripper, ON_NEW_DATA));

  msg_finished_action.data = true;
}

void loop() {
  // MicroROS - Arduino runs
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_move, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_gripper, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_finished_action, RCL_MS_TO_NS(100)));
}
