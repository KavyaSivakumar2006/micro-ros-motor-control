/*
 * ESP32 Motor Controller for QBotix 6-Wheeled Rover
 * Using micro_ros_arduino with Cytron MDS40B in Locked Antiphase Mode
 * 
 * Motor Pins:
 *   Left Side:  L1=25, L2=26, L3=19
 *   Right Side: R1=22, R2=18, R3=23
 * 
 * Locked Antiphase Mode:
 *   - 50% duty cycle (127) = STOP
 *   - 0% duty cycle (0) = Full REVERSE
 *   - 100% duty cycle (255) = Full FORWARD
 * 
 * Subscribes to: /motor_rpm (Float64MultiArray with [left_rpm, right_rpm])
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// ============================================================
// MOTOR PIN DEFINITIONS
// ============================================================
#define L1_PIN 25  // Left Front
#define L2_PIN 26  // Left Middle
#define L3_PIN 19  // Left Rear
#define R1_PIN 22  // Right Front
#define R2_PIN 18  // Right Middle
#define R3_PIN 23  // Right Rear

// PWM Configuration
#define PWM_FREQ 20000      // 20kHz PWM frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)
#define PWM_CHANNEL_L1 0
#define PWM_CHANNEL_L2 1
#define PWM_CHANNEL_L3 2
#define PWM_CHANNEL_R1 3
#define PWM_CHANNEL_R2 4
#define PWM_CHANNEL_R3 5

// ============================================================
// MOTOR PARAMETERS
// ============================================================
#define MAX_RPM 150.0       // Maximum motor RPM
#define STOP_PWM 127        // 50% duty cycle = stop in Locked Antiphase
#define MAX_PWM 255         // Maximum PWM value
#define MIN_PWM 0           // Minimum PWM value

// Robot parameters (must match ROS side)
#define WHEEL_RADIUS 0.10   // meters
#define WHEEL_SEPARATION 0.53  // meters

// Safety
#define CMD_TIMEOUT_MS 500  // Stop if no command for 500ms
#define RAMP_RATE 5         // PWM change per update cycle (smooth acceleration)

// ============================================================
// MICRO-ROS OBJECTS
// ============================================================
rcl_subscription_t motor_rpm_sub;
rcl_subscription_t cmd_vel_sub;
std_msgs__msg__Float64MultiArray motor_rpm_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ============================================================
// STATE VARIABLES
// ============================================================
volatile float target_left_rpm = 0.0;
volatile float target_right_rpm = 0.0;
volatile unsigned long last_cmd_time = 0;

int current_left_pwm = STOP_PWM;
int current_right_pwm = STOP_PWM;
int target_left_pwm = STOP_PWM;
int target_right_pwm = STOP_PWM;

bool micro_ros_connected = false;

// ============================================================
// ERROR HANDLING MACROS
// ============================================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    // Blink LED to indicate error
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ============================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================

/**
 * Initialize PWM channels for all motors
 */
void setupMotors() {
  // Configure PWM channels
  ledcSetup(PWM_CHANNEL_L1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R3, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach PWM channels to pins
  ledcAttachPin(L1_PIN, PWM_CHANNEL_L1);
  ledcAttachPin(L2_PIN, PWM_CHANNEL_L2);
  ledcAttachPin(L3_PIN, PWM_CHANNEL_L3);
  ledcAttachPin(R1_PIN, PWM_CHANNEL_R1);
  ledcAttachPin(R2_PIN, PWM_CHANNEL_R2);
  ledcAttachPin(R3_PIN, PWM_CHANNEL_R3);
  
  // Initialize all motors to STOP
  stopAllMotors();
}

/**
 * Stop all motors (set to 50% duty cycle for Locked Antiphase)
 */
void stopAllMotors() {
  ledcWrite(PWM_CHANNEL_L1, STOP_PWM);
  ledcWrite(PWM_CHANNEL_L2, STOP_PWM);
  ledcWrite(PWM_CHANNEL_L3, STOP_PWM);
  ledcWrite(PWM_CHANNEL_R1, STOP_PWM);
  ledcWrite(PWM_CHANNEL_R2, STOP_PWM);
  ledcWrite(PWM_CHANNEL_R3, STOP_PWM);
  
  current_left_pwm = STOP_PWM;
  current_right_pwm = STOP_PWM;
  target_left_pwm = STOP_PWM;
  target_right_pwm = STOP_PWM;
}

/**
 * Convert RPM to PWM value for Locked Antiphase mode
 * RPM > 0 = Forward (PWM 128-255)
 * RPM < 0 = Reverse (PWM 0-126)
 * RPM = 0 = Stop (PWM 127)
 */
int rpmToPwm(float rpm) {
  // Clamp RPM to valid range
  rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
  
  // Convert RPM to PWM
  // In Locked Antiphase:
  // - PWM 127 = stop
  // - PWM 255 = full forward (+MAX_RPM)
  // - PWM 0 = full reverse (-MAX_RPM)
  
  int pwm = STOP_PWM + (int)((rpm / MAX_RPM) * (STOP_PWM));
  
  // Clamp to valid PWM range
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  
  return pwm;
}

/**
 * Apply smooth ramping to motor PWM
 */
int rampPwm(int current, int target) {
  if (current < target) {
    return min(current + RAMP_RATE, target);
  } else if (current > target) {
    return max(current - RAMP_RATE, target);
  }
  return current;
}

/**
 * Set motor speeds with ramping
 */
void setMotorSpeeds(int left_pwm, int right_pwm) {
  // Apply ramping for smooth acceleration
  current_left_pwm = rampPwm(current_left_pwm, left_pwm);
  current_right_pwm = rampPwm(current_right_pwm, right_pwm);
  
  // Set left side motors
  ledcWrite(PWM_CHANNEL_L1, current_left_pwm);
  ledcWrite(PWM_CHANNEL_L2, current_left_pwm);
  ledcWrite(PWM_CHANNEL_L3, current_left_pwm);
  
  // Set right side motors
  ledcWrite(PWM_CHANNEL_R1, current_right_pwm);
  ledcWrite(PWM_CHANNEL_R2, current_right_pwm);
  ledcWrite(PWM_CHANNEL_R3, current_right_pwm);
}

// ============================================================
// ROS CALLBACKS
// ============================================================

/**
 * Callback for /motor_rpm topic
 * Receives [left_rpm, right_rpm] from skid_steer_controller
 */
void motorRpmCallback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  
  if (msg->data.size >= 2) {
    target_left_rpm = msg->data.data[0];
    target_right_rpm = msg->data.data[1];
    last_cmd_time = millis();
    
    // Convert RPM to PWM
    target_left_pwm = rpmToPwm(target_left_rpm);
    target_right_pwm = rpmToPwm(target_right_rpm);
  }
}

/**
 * Callback for /cmd_vel topic (alternative input)
 * Converts Twist to motor RPM directly
 */
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_vel = msg->linear.x;
  float angular_vel = msg->angular.z;
  
  // Differential drive kinematics
  float left_vel = linear_vel - (angular_vel * WHEEL_SEPARATION / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_SEPARATION / 2.0);
  
  // Convert velocity (m/s) to RPM
  // RPM = (velocity * 60) / (2 * PI * wheel_radius)
  target_left_rpm = (left_vel * 60.0) / (2.0 * PI * WHEEL_RADIUS);
  target_right_rpm = (right_vel * 60.0) / (2.0 * PI * WHEEL_RADIUS);
  
  last_cmd_time = millis();
  
  // Convert RPM to PWM
  target_left_pwm = rpmToPwm(target_left_rpm);
  target_right_pwm = rpmToPwm(target_right_rpm);
}

// ============================================================
// MICRO-ROS SETUP
// ============================================================

void setupMicroROS() {
  // Set micro-ROS transports (WiFi or Serial)
  // For WiFi:
  // set_microros_wifi_transports("SSID", "PASSWORD", "AGENT_IP", 8888);
  
  // For Serial (USB):
  set_microros_transports();
  
  delay(2000);  // Wait for agent connection
  
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));
  
  // Create /motor_rpm subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_rpm_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/motor_rpm"
  ));
  
  // Create /cmd_vel subscriber (backup/alternative)
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  ));
  
  // Initialize message memory
  motor_rpm_msg.data.capacity = 2;
  motor_rpm_msg.data.size = 0;
  motor_rpm_msg.data.data = (double *)malloc(2 * sizeof(double));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_rpm_sub, &motor_rpm_msg, &motorRpmCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  
  micro_ros_connected = true;
}

// ============================================================
// MAIN SETUP AND LOOP
// ============================================================

void setup() {
  // Initialize LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Initialize Serial for debugging (optional)
  Serial.begin(115200);
  Serial.println("QBotix ESP32 Motor Controller Starting...");
  
  // Initialize motors
  setupMotors();
  Serial.println("Motors initialized - Locked Antiphase Mode");
  
  // Initialize micro-ROS
  setupMicroROS();
  Serial.println("micro-ROS initialized");
  
  digitalWrite(LED_BUILTIN, LOW);  // LED off = running normally
}

void loop() {
  // Spin executor to process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // Check for command timeout (safety stop)
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    // No command received - stop motors
    target_left_pwm = STOP_PWM;
    target_right_pwm = STOP_PWM;
  }
  
  // Update motor speeds with ramping
  setMotorSpeeds(target_left_pwm, target_right_pwm);
  
  // Small delay for loop timing
  delay(10);  // ~100Hz control loop
}

// ============================================================
// WIFI VERSION (Alternative setup for WiFi transport)
// ============================================================
/*
 * To use WiFi instead of Serial, replace setupMicroROS() with:
 * 
 * void setupMicroROS() {
 *   // WiFi connection to micro-ROS agent
 *   set_microros_wifi_transports(
 *     "YOUR_WIFI_SSID",
 *     "YOUR_WIFI_PASSWORD", 
 *     "192.168.1.100",  // IP of computer running micro-ROS agent
 *     8888              // Agent port
 *   );
 *   
 *   ... rest of setup code ...
 * }
 * 
 * Then run the agent on your computer:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 */
