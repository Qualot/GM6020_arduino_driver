#include <SPI.h>
#include <mcp2515.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

// CAN setup
struct can_frame canMsgReceive;
struct can_frame canMsgSend;
MCP2515 mcp2515(10);

short power = -3000;
float rotation = 0;
float rotation_previous = 0;
float rotation_cumulative = 0;
float rotation_initial = 0;
short speed = 0;
short torque = 0;
int8_t temperature = 0;

// Sine wave parameters
float amplitude = 25.0;
float frequency = 4.0;
int num_cycles = 2000;
unsigned long startTime = 0;
bool sine_control_active = false;

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32MultiArray sensor_data_msg;
ros::Publisher sensor_data_pub("sensor_data", &sensor_data_msg);

// Callback function for Float32MultiArray topic
void paramCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 3) {
    amplitude = msg.data[0];
    frequency = msg.data[1];
    num_cycles = (int)msg.data[2];
    rotation_initial = rotation_cumulative;
    //rotation_initial = ((int)rotation_initial + 60) % 360;
    //rotation_initial = (float)rotation_initial;


    startTime = millis();
    sine_control_active = true;
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> param_sub("sine_wave_params", &paramCallback);

void sendValues(float target_position);
void readValues();

void setup() {
  SPI.begin();
  
  canMsgSend.can_id  = 0x1ff;
  canMsgSend.can_dlc = 8;
  memset(canMsgSend.data, 0, sizeof(canMsgSend.data));

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensor_data_pub);
  nh.subscribe(param_sub);
}

void loop() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;
  float target_position = 0.0;

  if (sine_control_active) {
    if (elapsedTime < num_cycles / frequency) {
      //target_position = rotation_initial;
      target_position = amplitude * sin(2 * M_PI * frequency * elapsedTime);
      //target_position = amplitude * sin(2 * M_PI * frequency * elapsedTime) + rotation_initial;
      //target_position = amplitude * (1.0 - cos(2 * M_PI * frequency * elapsedTime));
      //target_position = amplitude * (1.0 - cos(2 * M_PI * frequency * elapsedTime)) + rotation_initial;
      sendValues(target_position);
    } else {
      sendValues(0);
      sine_control_active = false;
    }
  }

  rotation_previous = rotation;
  readValues();
  
  float delta = rotation - rotation_previous;
  if (delta > 180.0) {
    delta -= 360.0;
  } else if (delta < -180.0) {
    delta += 360.0;
  }
  rotation_cumulative += delta;

  // Publish sensor data as a single Float32MultiArray
  sensor_data_msg.data_length = 6;
  sensor_data_msg.data = (float*)malloc(sensor_data_msg.data_length * sizeof(float));
  sensor_data_msg.data[0] = rotation;
  sensor_data_msg.data[1] = target_position;
  sensor_data_msg.data[2] = rotation_cumulative;
  sensor_data_msg.data[3] = speed;
  sensor_data_msg.data[4] = torque;
  sensor_data_msg.data[5] = (float)temperature;

  sensor_data_pub.publish(&sensor_data_msg);

  nh.spinOnce();
  delay(10);

  free(sensor_data_msg.data);  // Free the memory after publishing
}

void readValues() {
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {
    if (canMsgReceive.can_id == 0x205) {
      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
      rotation = (float)r / 8192 * 360;

      speed = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);
      torque = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);
      temperature = canMsgReceive.data[6];
    }
  }
}

void sendValues(float target_position) {
  short output = (target_position - rotation_cumulative) * 200; // 200 worked w/o divergence

  canMsgSend.data[0] = (output >> 8) & 0xFF;
  canMsgSend.data[1] = output & 0xFF;

  mcp2515.sendMessage(&canMsgSend);
}
