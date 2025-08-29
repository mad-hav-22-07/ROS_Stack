#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <QuadDecoder.h>

#define PULSES_PER_REV 1024 // Number of ticks per wheel revolution
#define COUNTS_PER_REV 4096
#define WHEEL_RADIUS 0.125
#define WHEEL_DIST 0.56
#define TIME_PERIOD_US 1e4 // Time interval between data, in microseconds. 10,000 microseconds == 100Hz (10ms)

QuadDecoder<1> Enc1(1); //New Quadrature Encoder Object
QuadDecoder<2> Enc2(1); //New Quadrature Encoder Object

volatile uint32_t seq = 1;

volatile unsigned long prev_time = 0;
volatile unsigned long curr_time = 0;
volatile float odom_time_prev = 0;

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN LED_BUILTIN
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
bool micro_ros_init_successful;

rcl_publisher_t enc_RPM;
std_msgs__msg__Float64MultiArray enc_RPM_msg;
rcl_publisher_t odom;
nav_msgs__msg__Odometry odom_msg;

volatile float pulses[2] = {0.0, 0.0};
volatile float rpm[2] = {0.0, 0.0};
volatile uint32_t tot_pulse[2] = {0,0};
volatile float yaw = 0.0;
volatile float x = 0.0;
volatile float y = 0.0;


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool create_entities()
{

  Serial.println("Agent connected");  
  
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "encoder_ethernet", "", &support));

  // Create message buffer
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2;
  
  static micro_ros_utilities_memory_conf_t conf2 = {0};
  
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    &enc_RPM_msg,
    conf
  );
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    &odom_msg,
    conf2
  );

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &enc_RPM,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "enc_RPM"));

  RCCHECK(rclc_publisher_init_default(
    &odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "Odom"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCSOFTCHECK(rmw_uros_sync_session(1000));

  return true;
}

void destroy_entities()
{
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2;
  micro_ros_utilities_destroy_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    &enc_RPM_msg,
    conf
  );

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  digitalWrite(LED_PIN, LOW);
}

void calculateOdom(){
  uint32_t cur_time_ns = rmw_uros_epoch_nanos();
  if (cur_time_ns == 0){
    Serial.println("Time not synchronized!");
  }

  odom_msg.header.stamp.nanosec = cur_time_ns;
  odom_msg.header.stamp.sec = (int32_t) (cur_time_ns / 1e9);
  odom_msg.header.frame_id.size = 5;
  odom_msg.header.frame_id.data = "odom";

  float odom_time_cur = (float)millis() / 1000;
  float dt = odom_time_cur - odom_time_prev;
  odom_time_prev = odom_time_cur;

  float th1 = (2*3.1415*tot_pulse[0]) / COUNTS_PER_REV;
  float th2 = (2*3.1415*tot_pulse[1]) / COUNTS_PER_REV;

  float s1 = th1 * WHEEL_RADIUS;
  float s2 = th2 * WHEEL_RADIUS;

  float ds = (s1 + s2) / 2.0;
  float dyaw = (s2 - s1) / (2.0 * WHEEL_DIST);
  float cos_yaw = cosf(yaw + dyaw);
  float sin_yaw = sinf(yaw + dyaw);

  float dx = ds * cos_yaw;
  float dy = ds * sin_yaw;

  float vx = dx / dt;
  float vy = dy / dt;
  float wz = dyaw / dt;

  yaw += dyaw;
  x += dx;
  y += dy;
  tot_pulse[0] = 0;
  tot_pulse[1] = 0;
  
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.w = cosf(yaw * 0.5);
  odom_msg.pose.pose.orientation.z = sinf(yaw * 0.5);
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = wz;
}

void setup() {
  Serial.begin(921600);
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };

  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 0, 177);
  IPAddress agent_ip(192, 168, 0, 100);
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(500);

  state = WAITING_AGENT;

  // Quadtrature decoder setup
	Enc1.begin(PULSES_PER_REV * 4, TIME_PERIOD_US);
	Enc2.begin(PULSES_PER_REV * 4, TIME_PERIOD_US);

  Serial.println("Setup finished");

  odom_time_prev = (float)millis() / 1000;

}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      Serial.println("Agent disconnected");
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  curr_time = micros();

  if((curr_time - prev_time >= 10000) && (state==AGENT_CONNECTED)){

    prev_time = curr_time;

    pulses[0] = Enc1.getDCount();
    pulses[1] = Enc2.getDCount();
    if (pulses[1] > 60000){
      pulses[1] = pulses[1] - 65536;
    }
    if (pulses[0] > 60000){
      pulses[0] = pulses[0] - 65536;
    }

    tot_pulse[0] += (uint32_t)pulses[0];
    tot_pulse[1] += (uint32_t)pulses[1];

    enc_RPM_msg.layout.data_offset = 0;
    enc_RPM_msg.data.size = 2;

    rpm[0] = Enc1.getVelocity();
    rpm[1] = Enc2.getVelocity();

    enc_RPM_msg.data.data[0] = rpm[0];
    enc_RPM_msg.data.data[1] = rpm[1];
    
    RCSOFTCHECK(rcl_publish(&enc_RPM, &enc_RPM_msg, NULL));

    seq++;

    if (seq == 10){
      calculateOdom();
      RCSOFTCHECK(rcl_publish(&odom, &odom_msg, NULL));
      seq = 0;
    }

    digitalWrite(LED_PIN, HIGH);
    // Serial.println("data sent");

  }
  else if(curr_time - prev_time >= 10000){
    prev_time = curr_time;
  }
}
