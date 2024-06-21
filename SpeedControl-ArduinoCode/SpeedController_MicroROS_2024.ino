#include <micro_ros_arduino.h>
#include <SPI.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// Pins
#define PUBLISH_INTERVAL 500 // Publish interval in milliseconds
#define TIMEOUT_INTERVAL 1000 // Timeout interval in milliseconds
#define ENC_IN A1 //interrupt
#define FORWARDIN 1
#define BACKWARDIN 2
#define SAFTY_IN 3
#define FORWARDOUT 5
#define BACKWARDOUT 6 
#define SAFETY_OUT 7
#define NOISE_THRESHOLD 20 // Adjust this threshold as needed

byte address = 0x00;
int CS = 4;
int i = 0;
int j = 0;
float ENC_COUNT_REV = 48;
float encoderValue = 0;
long currentMillis = 0;
long previousMillis = 0;
int interval = 500;
float rpm = 0;
float KmperHr = 0;
float DperR = 1.41/1000;
int stateFW = 0 ;
int stateBW = 0 ;
int statePedal = 0 ;
int adjust_speed = 0, adjust_8bit = 0; //set initial to 0 V
int bit;
bool countENC = false ;
bool start = false ;
int previousState = LOW;
int currentState = LOW;
float vall[11];
int count = 0;
float currentSpeed, result, AvgSpeed;
float setSpeedd = 0;
float Kprop = 1, Kintegral = 0.00113, Kderiv = 3.35;
float sigmaerror = 0, dT, preverror, prev = 0, adjust, prevadjust;
// ROS variables
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg_pub;
std_msgs__msg__Float32 msg_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

unsigned long last_received_time = 0;
unsigned long last_publish_time = 0;
float received_value = 0.0;

void subscription_callback(const void *msg_in) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
  received_value = msg->data;
  last_received_time = millis();
  countENC = true;
  encoderValue = 0;
  setSpeedd = received_value;
}

void setup() {
  Serial.begin(9600);
  set_microros_transports();

  // Initialize allocator
  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "arduino_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/actual_speed");

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/desired_speed");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);

  pinMode(FORWARDIN, INPUT);
  pinMode(BACKWARDIN, INPUT);
  pinMode(SAFTY_IN, INPUT);
  pinMode(FORWARDOUT, OUTPUT);
  pinMode(BACKWARDOUT, OUTPUT);
  pinMode(SAFETY_OUT, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(ENC_IN, INPUT);
  SPI.begin();
  digitalPotWrite(0);
  previousMillis = millis();
}

void updateEncoder() {
  currentState = analogRead(ENC_IN);
  if (currentState > 400 && previousState < 80) {
    encoderValue++; // Increment count on rising edge
  }
  previousState = currentState;
}

void loop() {
  // Publish data at regular intervals
  updateEncoder();
  unsigned long current_time = millis();
  currentMillis = millis();

  if (countENC) {
    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      stateFW = digitalRead(FORWARDIN);
      stateBW = digitalRead(BACKWARDIN);
      statePedal = 1;

      rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
      KmperHr = (float)(((rpm * 60) * DperR) * 2);
      averagefilter(KmperHr);
      j++;

      currentSpeed = KmperHr;

      if (j < 20) {
        AvgSpeed = KmperHr;
      } 
      else {
        AvgSpeed = averagefilter(KmperHr);
      }

      Serial.print(j);
      Serial.print(",");
      Serial.print(encoderValue);
      Serial.print(",");
      Serial.print(KmperHr);
      Serial.print(",");
      Serial.println(AvgSpeed);

      adjust = SpeedControl(currentSpeed, setSpeedd, currentMillis, statePedal);
      if (adjust > 100) {
        adjust = 100;
      }

      autospeed();
      msg_pub.data = AvgSpeed; 
      rcl_publish(&publisher, &msg_pub, NULL);
      Serial.println(msg_pub.data);
      encoderValue = 0;
      
    }
  }
  else {
    digitalWrite(SAFETY_OUT, LOW);
    digitalWrite(FORWARDOUT, LOW);
    digitalPotWrite(0);
  }

  if (countENC && (current_time - last_received_time >= TIMEOUT_INTERVAL)) {
    Serial.println("No message received within the timeout interval");
    countENC = false; // Reset the flag
    msg_pub.data = 0.0; 
    rcl_publish(&publisher, &msg_pub, NULL);
    KmperHr = 0;
    j=0;
    preverror = 0;
    prevadjust = 0;
    prev=0;
    dT=0;
    sigmaerror= 0;
    currentMillis = 0;
    encoderValue = 0;
  }

  // Handle incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0.1));
}

float SpeedControl(float current, float set, float currentTime, float state) {
  float error = set - current;
  dT = 500; 
  prev = currentTime;

  if (state == 1) {
    sigmaerror = sigmaerror + (error * dT);
    adjust_speed = (error * Kprop) + (Kintegral * sigmaerror) + (Kderiv * ((error - preverror) / dT));

    if (error < 0.2 && error > -0.2 && preverror < 0.3 && preverror > -0.3) {
      adjust_speed = prevadjust;
    }
  } 
  else {
    sigmaerror = 0;
    adjust_speed = 0;
  }

  preverror = error;
  prevadjust = adjust_speed;
  adjust_8bit = 2.8438 * adjust_speed + 5.2142;

  if (adjust_8bit <= 13) {
    adjust_8bit = 13;
  }

  return adjust_8bit;
}

void autospeed() {
  digitalWrite(SAFETY_OUT, HIGH);
  digitalWrite(FORWARDOUT, HIGH);
  delay(50);
  digitalPotWrite(adjust);
}

float averagefilter(float kmh) {
  float sum = 0;

  if (count == 11) {
    count = 0;
  } else {
    vall[count] = kmh;
  }

  for (int i = 0; i < 11; i++) {
    sum += vall[i];
  }

  result = sum / 10;
  count++;
  return result;
}

void digitalPotWrite(byte value) {
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

