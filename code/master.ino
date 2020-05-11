//Main sensor head
//Three SR04 sonar sensors
//One SPAD, rotated by a servo
//Serial feed to bluetooth for debug
//Serial feed to main processor for drive gear

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#define MAX_WAIT 10
int led = 13;
int ctr = 0;
int servo_pin1 = 15;
int sonarTrig1 = 5;
int sonarTrig2 = 4;
int sonarTrig3 = 3;
int sonarEcho1 = 8;
int sonarEcho2 = 7;
int sonarEcho3 = 6;

Servo servo1;
VL53L0X sensor;
uint16_t sensor_map[181];

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  pinMode(sonarTrig1, OUTPUT); //Forward facing sonar
  pinMode(sonarTrig2, OUTPUT); //Left facing sonar
  pinMode(sonarTrig3, OUTPUT); //Right facing sonar
  pinMode(sonarEcho1, INPUT);
  pinMode(sonarEcho2, INPUT);
  pinMode(sonarEcho3, INPUT);
  Wire.begin();
  Serial1.begin(9600);
  Serial2.begin(115200);
  servo1.attach(servo_pin1);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial1.println("Sensor detection failed");
    Serial1.flush();
    while (1);
  }
  sensor.setMeasurementTimingBudget(20000);
  delay(2000);
  digitalWrite(led, LOW);
}

void loop() {
  int k, forward, left, right;
  forward = ping(sonarTrig1, sonarEcho1);
  left = ping(sonarTrig2, sonarEcho2);
  right = ping(sonarTrig3, sonarEcho3);
  if (forward >= left && forward >= right) { //Bias forwardd (90-5)
    servo1.write(125);
    delay(50);
    for (k = 125; k >= 55; k--) updateScanMap(k, forward);
    sendLongestRange(55, 125);
  }
  else if (left >= forward && left >= right) { //Bias to the left (180-10)
    servo1.write(180);
    delay(50);
    for (k = 180; k >= 110; k--) updateScanMap(k, left);
    sendLongestRange(110, 180);
  }
  else { //Bias right (0)
    servo1.write(70);
    delay(50);
    for (k = 70; k >= 0; k--) updateScanMap(k, right);
    sendLongestRange(0, 70);
  }
}

void sendToSlave(String l) {
  String ack;
  int wait_count;
  //Send the string value
  digitalWrite(led, HIGH);
  Serial2.print(l);
  Serial2.flush();
  //Wait for it to be returned as an ACK
  wait_count = 0;
  while (ack != l) {
    while (Serial2.available() == 0) {
      delay(200);
      wait_count++;
      //If an ACK hasn't been seen for a while, resend.
      if (wait_count > MAX_WAIT) {
        Serial2.print(l);
        Serial2.flush();
        Serial1.print("Resending \"");
        Serial1.print(l);
        Serial1.println("\"");
        Serial1.flush();
        wait_count = 0;
      }
    }
    //Serial data available, read expected ACK value
    while (Serial2.available() > 0) {
      ack = Serial2.readString();
    }
    //If ACT doesn't match what was sent, re-send value
    if (String(l) != ack) {
      Serial2.print(l);
      Serial2.flush();
    }

  }
  digitalWrite(led, LOW);
  return;
}
void sendLongestRange(int left, int right) {
  String rangePair;
  int l;
  int large = 0;
  uint16_t biggest = 0;
  for (l = left; l <= right; l++) {
    if (sensor_map[l] > biggest) {
      large = l;
      biggest = sensor_map[l];
    }
  }
  rangePair = String(large);
  rangePair.concat(",");
  rangePair.concat(biggest);
  //Serial.print("Best=");
  //Serial.println(rangePair);
  //Serial.flush();
  sendToSlave(rangePair);

}


void updateScanMap(int k, int sonarDist) {
  uint16_t distance;
  servo1.write(k);
  distance = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial1.println("Sensor timeout occurred");
    Serial1.flush();
    sensor_map[k] = 0;
  }
  else {
    if (distance > 2000) distance = sonarDist;
    sensor_map[k] = distance;
  }
  Serial1.print("Deg=");
  Serial1.print(k);
  Serial1.print(" Range=");
  Serial1.print(" ");
  Serial1.println(sensor_map[k]);
  Serial1.flush();
}
int ping(int trigPin, int echoPin) {
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.34 / 2;
  return distance;
}
