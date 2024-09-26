#include <SPI.h>
#include <mcp2515.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

// CAN setup
struct can_frame canMsgReceive;
struct can_frame canMsgSend;
MCP2515 mcp2515(10);

int8_t controlmode = 8; //0...len, 1...stop, 2...direct, 4...vibration, 8...0out
int8_t controlmode_previous = 8;

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

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32MultiArray sensor_data_msg;
ros::Publisher sensor_data_pub("sensor_data", &sensor_data_msg);

void sendValues(short);
void readValues();
void shiftControlMode(int8_t);

// Callback function for Float32MultiArray topic
void paramCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 4) {
    //controlmode = (int)msg.data[0];
    shiftControlMode((int)msg.data[0]);
    amplitude = msg.data[1];
    frequency = msg.data[2];
    num_cycles = (int)msg.data[3];
    rotation_initial = rotation_cumulative;

    startTime = millis();
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> param_sub("sine_wave_params", &paramCallback);

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

  sensor_data_msg.data = (float*)malloc(sensor_data_msg.data_length * sizeof(float));
  sensor_data_msg.data_length = 9;

  readValues();
  rotation_cumulative = rotation;
  rotation_initial = rotation;
}

void loop() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;
  float target_position = 0.0;
  float pgain = 200.0;
  short output = 0;

  switch (controlmode)
  {
  case 0: //len
    target_position = amplitude;
    output = (target_position - rotation_cumulative) * pgain; // pgain = 200 worked w/o divergence
    sendValues(output);
    break;

  case 1: //stop
    if (controlmode != controlmode_previous) {
      controlmode_previous = controlmode;
      rotation_initial = rotation_cumulative;
    }
    target_position = rotation_initial;
    output = (target_position - rotation_cumulative) * pgain; // pgain = 200 worked w/o divergence
    sendValues(output);
    break;

  case 2: //direct
    output = amplitude;
    sendValues(output);
    break;

  case 4: //vibration
    if (elapsedTime < num_cycles / frequency) {
      //target_position = amplitude * sin(2 * M_PI * frequency * elapsedTime);
      target_position = amplitude * (1.0 - cos(2 * M_PI * frequency * elapsedTime)) + rotation_initial;
      output = (target_position - rotation_cumulative) * pgain; // pgain = 200 worked w/o divergence
      sendValues(output);
    } else {
      shiftControlMode(1);
    }
    break;

  case 8: //0out
    sendValues(0);
    break;
  
  default:
    sendValues(0);
    break;
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
  sensor_data_msg.data[0] = rotation_initial;
  sensor_data_msg.data[1] = target_position;
  sensor_data_msg.data[2] = rotation_cumulative;
  sensor_data_msg.data[3] = output;
  sensor_data_msg.data[4] = speed;
  sensor_data_msg.data[5] = torque;
  sensor_data_msg.data[6] = (float)temperature;
  sensor_data_msg.data[7] = (float)controlmode;
  sensor_data_msg.data[8] = (float)controlmode_previous;

  sensor_data_pub.publish(&sensor_data_msg);

  nh.spinOnce();
  delay(10);

  //free(sensor_data_msg.data);  // Free the memory after publishing
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

void sendValues(short output) {
  canMsgSend.data[0] = (output >> 8) & 0xFF;
  canMsgSend.data[1] = output & 0xFF;
  mcp2515.sendMessage(&canMsgSend);
}

void shiftControlMode(int8_t mode) {
  if (mode ==1 || mode == 2 || mode == 4 || mode == 8) {
    controlmode_previous = controlmode;
    controlmode = mode;
  }
}
