/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port.
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>
#include <PulsePosition.h>
#include <WS2812Serial.h>

const int numled = 136;
const int pin = 29;

byte drawingMemory[numled * 3];       //  3 bytes per LED
DMAMEM byte displayMemory[numled * 12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);
PulsePositionInput ReceiverInput(RISING);

//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

#define Wheel_Distance 0.5
#define Wheel_Circumference 0.798
#define RPM_Click 140
#define Encoder_Click 840
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xE25822

ros::NodeHandle nh;
int colours[7] = {RED, GREEN, BLUE, YELLOW, PINK, ORANGE, WHITE};
//geometry_msgs::TransformStamped t;
std_msgs::Float64 batt_msg;
std_msgs::Float64MultiArray motortemp_msg;
nav_msgs::Odometry odom_msg;
char baseFrame[] = "base_link";
char odomFrame[] = "odom";

/** Initiate VescUart class */
VescUart MotorFR;
VescUart MotorFL;
VescUart MotorBL;
VescUart MotorBR;

bool colour_flag = false;
float ReceiverValue[] = {1500, 1500, 1500, 1500, 0, 0, 0, 0};
float i_float = 0.0;
float k_float = 0.0;
int ChannelNumber = -1;
float left_rpm = 0;
float right_rpm = 0;
int left_wheel_joy = 1500;
int right_wheel_joy = 1500;
float left_speed_ms = 0;
float right_speed_ms = 0;
int spd = 500;
int turn = 1000;
float cmd_vx = 0, cmd_vy = 0, cmd_vz = 0;
int32_t current_FR_enc;
int32_t current_FL_enc;
int32_t current_BR_enc;
int32_t current_BL_enc;
float prev_timer;
float delta_timer;
float lasttime = millis();
float THETA;
float delta_x;
float delta_y;
float x;
float y;
int32_t  prev_FL_enc = 0;
int32_t  prev_FR_enc = 0;
int32_t  prev_BL_enc = 0;
int32_t  prev_BR_enc = 0;
float FR_temp = 0.0;
float BR_temp = 0.0;
float FL_temp = 0.0;
float BL_temp = 0.0;
float rain_spd = 0.0;
int channel3_val = 0;
int m = 0;

void cmdVelCb(const geometry_msgs::Twist& msg) {
  cmd_vx = msg.linear.x; // m/s
  cmd_vy = 0.0; // m/s
  cmd_vz = msg.angular.z; // rad/s
  Serial.println("Recieve cmd vel");
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);
ros::Publisher battery("battery", &batt_msg);
ros::Publisher motortemp("motor_temp", &motortemp_msg);
ros::Publisher odomPub("odom_raw", &odom_msg);

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(57600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  ReceiverInput.begin(6);

  nh.getHardware()->setBaud(57600);

  //broadcaster.init(nh);
  nh.initNode();
  nh.subscribe(cmdVelSub);
  nh.advertise(battery);
  nh.advertise(motortemp);
  nh.advertise(odomPub);
  motortemp_msg.data = (float*)malloc(sizeof(float) * 4);
  motortemp_msg.data_length = 4;
  while (!Serial2) {
    ;
    Serial2.println("Front Right Serial Fail");
  }
  while (!Serial3) {
    ;
    Serial3.println("Front Left Serial Fail");
  }
  while (!Serial4) {
    ;
    Serial4.println("Back Left Serial Fail");
  }
  while (!Serial5) {
    ;
    Serial5.println("Back Right Serial Fail");
  }

  /** Define which ports to use as UART */
  MotorFL.setSerialPort(&Serial2);
  MotorFR.setSerialPort(&Serial3);
  MotorBL.setSerialPort(&Serial4);
  MotorBR.setSerialPort(&Serial5);

  delay(3000);

  if ( MotorFR.getVescValues() ) {
    prev_FR_enc = MotorFR.data.tachometer;
  }
  if ( MotorFL.getVescValues() ) {
    prev_FL_enc = MotorFL.data.tachometer;
  }
  if ( MotorBR.getVescValues() ) {
    prev_BR_enc = MotorBR.data.tachometer;
  }
  if ( MotorBL.getVescValues() ) {
    prev_BL_enc = MotorBL.data.tachometer;
  }

  leds.begin();
  leds.setBrightness(40);
  leds.show();
  prev_timer = millis();
}

void loop() {
  getVESCValue();
  batt_msg.data = analogRead(A9) / 35.2;
  Serial.println(batt_msg.data);
  battery.publish( &batt_msg);
  //Serial.println(FL_temp);
  motortemp_msg.data[0] = FL_temp;
  motortemp_msg.data[1] = FR_temp;
  motortemp_msg.data[2] = BL_temp;
  motortemp_msg.data[3] = BR_temp;

  motortemp.publish( &motortemp_msg);
  read_receiver();

  int knob1_val = ReceiverValue[4];
  int knob2_val = ReceiverValue[5];
  rain_spd = (knob1_val - 1000) / 1000.0 * 2.5;
  if (knob2_val >= 999 && knob2_val < 1100) {
    Serial.println("Rainbow colour wipe");
    if (colour_flag == true) {
      clear_pixel();
    }
    colour_flag = false;
    colorWipe(Wheel(m), rain_spd, 15);
    m++;
    if (m > 765) {
      m = 0;
    }
    Serial.print("m:");
    Serial.println(m);
  }

  else if (knob2_val > 1100 && knob2_val < 1900) {
    if (colour_flag == true) {
      clear_pixel();
    }
    colour_flag = false;
    int colour_ind = map(knob2_val, 1100, 1900, 0, 765);
    colorWipe(Wheel(colour_ind), rain_spd, 15);
    Serial.print("colour_ind:");
    Serial.println(colour_ind);
  }
  else {
    Serial.println("Rainbow cycle");
    rainbowCycle(rain_spd * 8);
    colour_flag = true;
  }

  channel3_val = ReceiverValue[3];

  if (!(channel3_val > 1250 && channel3_val < 1750)) {
    Serial.println("Activated");
    left_wheel_joy = ReceiverValue[1];
    right_wheel_joy = ReceiverValue[0];
    if (left_wheel_joy >= 1497 && left_wheel_joy <= 1503)
      left_wheel_joy = 1500;
    if (right_wheel_joy >= 1497 && right_wheel_joy <= 1503)
      right_wheel_joy = 1500;
    //    left_speed_ms = (left_wheel_joy - 2000) * 1.5 / (-1000) - 0.75;
    //    right_speed_ms = (right_wheel_joy - 2000) * 1.5 / (-1000) - 0.75;
    left_speed_ms = map(float(left_wheel_joy), 1000.0, 2000.0, -0.75, 0.75);
    right_speed_ms = map(float(right_wheel_joy), 1000.0, 2000.0, -0.75, 0.75);
    right_rpm = ( right_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
    left_rpm = ( left_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
    if (right_rpm == 0) {
      MotorBR.setBrakeCurrent(30);
      MotorFR.setBrakeCurrent(30);
      MotorBL.setBrakeCurrent(30);
      MotorFL.setBrakeCurrent(30);
      //MotorBR.setRPM(0);
      //MotorFR.setRPM(0);
    }
    else {
      MotorBR.setRPM(right_rpm);
      MotorFR.setRPM(right_rpm);
    }
    if (left_rpm == 0) {
      MotorFL.setBrakeCurrent(30);
      MotorBL.setBrakeCurrent(30);
      MotorBR.setBrakeCurrent(30);
      MotorFR.setBrakeCurrent(30);
      //MotorFL.setRPM(0);
      //MotorBL.setRPM(0);
    }
    else {
      MotorFL.setRPM(left_rpm);
      MotorBL.setRPM(left_rpm);
    }
  }
  else {
    MotorFL.setBrakeCurrent(30);
    MotorBL.setBrakeCurrent(30);
    MotorFR.setBrakeCurrent(30);
    MotorBR.setBrakeCurrent(30);
  }

  Serial.print("left:");
  Serial.print(left_speed_ms);
  Serial.print(" right:");
  Serial.println(right_speed_ms);
  Serial.println(ChannelNumber);

  //    Serial.print("VX:");
  //    Serial.print(cmd_vx);
  //    Serial.print(" |VY:");
  //    Serial.print(cmd_vy);
  //    Serial.print(" |VZ:");
  //    Serial.println(cmd_vz);
  Odometry();

  nh.spinOnce();
}


void read_receiver(void) {
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i = 1; i <= ChannelNumber; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
  Serial.print("Channel: ");
  Serial.println(ChannelNumber);
}

void cmd_velToVESC() {
  float right_speed_ms = (cmd_vz * Wheel_Distance) / 2 + cmd_vx;
  float left_speed_ms = (cmd_vx * 2) - right_speed_ms;
  float right_rpm = ( right_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
  float left_rpm = ( left_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
  if (right_rpm == 0) {
    MotorBR.setBrakeCurrent(30);
    MotorFR.setBrakeCurrent(30);
    //MotorBR.setRPM(0);
    //MotorFR.setRPM(0);
  }
  else {
    MotorBR.setRPM(right_rpm);
    MotorFR.setRPM(right_rpm);
  }
  if (left_rpm == 0) {
    MotorFL.setBrakeCurrent(30);
    MotorBL.setBrakeCurrent(30);
    //MotorFL.setRPM(0);
    //MotorBL.setRPM(0);
  }
  else {
    MotorFL.setRPM(left_rpm);
    MotorBL.setRPM(left_rpm);
  }
}

void getVESCValue() {
  if ( MotorFR.getVescValues() ) {
    current_FR_enc = MotorFR.data.tachometer;
    FR_temp = MotorFR.data.tempMotor;
  }
  if ( MotorFL.getVescValues() ) {
    current_FL_enc = MotorFL.data.tachometer;
    FL_temp = MotorFL.data.tempMotor;
  }
  if ( MotorBR.getVescValues() ) {
    current_BR_enc = MotorBR.data.tachometer;
    BR_temp = MotorBR.data.tempMotor;
  }
  if ( MotorBL.getVescValues() ) {
    current_BL_enc = MotorBL.data.tachometer;
    BL_temp = MotorBL.data.tempMotor;
  }
}

void Odometry() {
  float FL_delta_enc = (current_FL_enc - prev_FL_enc);
  float FR_delta_enc = (current_FR_enc - prev_FR_enc);
  float BL_delta_enc = (current_BL_enc - prev_BL_enc);
  float BR_delta_enc = (current_BR_enc - prev_BR_enc);

  prev_FL_enc = current_FL_enc;
  prev_FR_enc = current_FR_enc;
  prev_BL_enc = current_BL_enc;
  prev_BR_enc = current_BR_enc;

  float left_delta_metre = (((FL_delta_enc + BL_delta_enc) / 2) / Encoder_Click ) * Wheel_Circumference;
  float right_delta_metre = (((FR_delta_enc + BR_delta_enc) / 2) / Encoder_Click ) * Wheel_Circumference;
  //  float left_delta_metre = (FL_delta_enc  / Encoder_Click ) * Wheel_Circumference;
  //  float right_delta_metre = (FR_delta_enc / Encoder_Click ) * Wheel_Circumference;
  float delta_r = (left_delta_metre + right_delta_metre) / 2.0;
  float delta_theta = (right_delta_metre - left_delta_metre) / Wheel_Distance; //angular velocity*/

  THETA = delta_theta + THETA;

  if (THETA >= 6.28) {
    THETA = THETA - 6.28;
  }
  if (THETA <= -6.28) {
    THETA = THETA + 6.28;
  }

  delta_x = delta_r * cos(THETA);
  delta_y = delta_r * sin(THETA);

  x = x + delta_x; //linear x  ??dk if need refer to diffposition.h
  y = y + delta_y; //linear y

  //    Serial.print("vx");
  //    Serial.print(vx);
  //    Serial.print(" theta:");
  //    Serial.println(vtheta);

  //  Serial.print("FR");
  //  Serial.print(current_FR_enc);
  //  Serial.print(" BR:");
  //  Serial.print(current_BR_enc);
  //  Serial.print(" FL:");
  //  Serial.print(current_FL_enc);
  //  Serial.print(" BL");
  //  Serial.println(current_BL_enc);
  //  Serial.print("left_ms:");
  //  Serial.print(left_speed_ms);
  //  Serial.print(" |right_ms:");
  //  Serial.print(right_speed_ms);
  delta_timer = (millis() - prev_timer) / 1000;
  prev_timer = millis();

  //  Serial.print(" |x:");
  //  Serial.print(x);
  //  Serial.print(" |y:");
  //  Serial.print(y);
  //  Serial.print(" |theta:");
  //  Serial.println(THETA);

  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(THETA / 2.0);
  quaternion.w = cos(THETA / 2.0);

  /* Publish the distances and speeds on the odom topic. Set the timestamp
      to the last encoder time. */
  odom_msg.header.frame_id = odomFrame;
  odom_msg.child_frame_id = baseFrame;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = quaternion;
  odom_msg.twist.twist.linear.x =  delta_r / delta_timer;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = delta_theta / delta_timer;
  odomPub.publish(&odom_msg);
}

void colorWipe(uint32_t c, float spd, int pix) {

  int  n = leds.numPixels() / 4;
  int on_ind[pix];
  int off_ind[pix];
  i_float += spd;
  int i = int(i_float);
  if (i_float > n)
    i_float = 0;

  for (int x = 0; x < pix; x++) {
    if (i - x < 0) {
      off_ind[x] = i - x + n;
    }
    else {
      off_ind[x] = i - x;
    }
  }

  for (int x = 0; x < pix; x++) {
    if (i + x < n) {
      on_ind[x] = i + x;
    }
    else {
      on_ind[x] = i + x - n;
    }
  }

  for (int x = 0; x < pix; x++) {
    leds.setPixelColor(n - on_ind[x], c);
    leds.setPixelColor(on_ind[x] + n, c);
    leds.setPixelColor(3 * n - on_ind[x], c);
    leds.setPixelColor(on_ind[x] + n * 3, c);
  }
  for (int x = 0; x < pix; x++) {
    leds.setPixelColor(n - off_ind[x], 0);
    leds.setPixelColor(off_ind[x] + n, 0);
    leds.setPixelColor(3 * n - off_ind[x], 0);
    leds.setPixelColor(off_ind[x] + n * 3, 0);
  }
  leds.show();
}

void colorWipeRGB(uint32_t c, float spd, int pix) {

  int  n = leds.numPixels() / 4;
  int on_ind[pix];
  int off_ind[pix];
  i_float += spd;
  int i = int(i_float);
  if (i_float > n)
    i_float = 0;

  for (int x = 0; x < pix; x++) {
    if (i - x < 0) {
      off_ind[x] = i - x + n;
    }
    else {
      off_ind[x] = i - x;
    }
  }

  for (int x = 0; x < pix; x++) {
    if (i + x < n) {
      on_ind[x] = i + x;
    }
    else {
      on_ind[x] = i + x - n;
    }
  }

  for (int x = 0; x < pix; x++) {
    leds.setPixelColor(n - on_ind[x], c);
    leds.setPixelColor(on_ind[x] + n, c);
    leds.setPixelColor(3 * n - on_ind[x], c);
    leds.setPixelColor(on_ind[x] + n * 3, c);
  }
  for (int x = 0; x < pix; x++) {
    leds.setPixelColor(n - off_ind[x], 0);
    leds.setPixelColor(off_ind[x] + n, 0);
    leds.setPixelColor(3 * n - off_ind[x], 0);
    leds.setPixelColor(off_ind[x] + n * 3, 0);
  }
  leds.show();
}



void rainbowCycle(float spd) {
  int n = leds.numPixels() / 4;
  k_float += spd;
  int k = int(k_float);
  if (k_float > 256 * 5)
    k_float = 0;
  for (int j = 0; j < n; j++) {
    leds.setPixelColor(j, Wheel_rainbow(((j * 256 / n) + k) & 255));
    leds.setPixelColor(2 * n - j, Wheel_rainbow(((j * 256 / n) + k) & 255));
    leds.setPixelColor(2 * n + j, Wheel_rainbow(((j * 256 / n) + k) & 255));
    leds.setPixelColor(4 * n - j, Wheel_rainbow(((j * 256 / n) + k) & 255));
  }
  leds.show();

}

uint32_t Wheel_rainbow(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return leds.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return leds.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return leds.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

uint32_t Wheel(int WheelPos) {
  WheelPos = 765 - WheelPos;
  if (WheelPos < 255) {
    return leds.Color(255 - WheelPos, 0, WheelPos);
  }
  if (WheelPos < 511) {
    WheelPos -= 255;
    return leds.Color(0, WheelPos, 255 - WheelPos);
  }
  WheelPos -= 510;
  return leds.Color(WheelPos, 255 - WheelPos, 0);
}

void clear_pixel() {
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, 0);
  }
  leds.show();

}
