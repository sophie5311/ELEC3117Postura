#include <Arduino.h>
#include "Timer.h"
#include <cmath>
#include <pwmWrite.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>


uint8_t hubAddress[] = {0xF0, 0x24, 0xF9, 0xE7, 0x24, 0xC8};

typedef struct outgoingBack {
  float pitch;
  float roll;
  bool badPosture;
  float neutralPitch;
  float neutralRoll;
  bool calibrationSuccessful;
  float kneeAngle;
} outgoingBack;

typedef struct incomingHub {
  bool back;
  bool knee;
  float timer;
  bool calibrate;
  bool vibMotor;
  float sensitivity;
} incomingHub;

outgoingBack myData;
incomingHub hubData;

esp_now_peer_info_t peerInfo;

Servo myservo = Servo();
Adafruit_MPU6050 mpu;
const int pushButton = 33;
const int motorPin = 4;
const int resolution = 8;
Timer timer;

bool backOn = false;
float timerDuration = 2000;
bool calibrating = false;
bool vibMotorOn = true;
float sensitivity = 5;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&hubData, incomingData, sizeof(hubData));
  Serial.println("DATA RECIEVED ");
  backOn = hubData.back;
  timerDuration = hubData.timer;
  calibrating = hubData.calibrate;
  vibMotorOn = hubData.vibMotor;
  sensitivity = hubData.sensitivity;
  Serial.print(backOn);
  Serial.print("sen: ");
  Serial.println(sensitivity);
}

void setup(void) {
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);
  pinMode(pushButton, INPUT);
  
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  while (!Serial) {
    delay(10); 
  }
  
  //setting up esp32 NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //setting up mpu6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {

  int sendBuffer = 0;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long lastSampleMicros = micros();
  sensors_vec_t neutralPos = a.acceleration;
  sensors_vec_t neutralgyro = g.gyro;
  float pitch = atan2(a.acceleration.x, sqrt(pow(a.acceleration.y, 2.0) + pow(a.acceleration.z, 2.0)));
  float roll = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2.0) + pow(a.acceleration.z, 2.0)));
  bool first = true;
  float neutralPitch;
  float neutralRoll;

  while (1) {

    int buttonState = digitalRead(pushButton);

    mpu.getEvent(&a, &g, &temp);
    float normalizedGyroscopeX = g.gyro.x / 32.8;
    float normalizedGyroscopeY = g.gyro.z / 32.8;
    float normalizedGyroscopeZ = g.gyro.y / 32.8;
    float normalizedAccelerometerX = a.acceleration.x * 9.80665 / 16384;
    float normalizedAccelerometerY = a.acceleration.z * 9.80665 / 16384;
    float normalizedAccelerometerZ = a.acceleration.y * 9.80665 / 16384;

    float accX = atan(normalizedAccelerometerY / sqrt(sq(normalizedAccelerometerX) + sq(normalizedAccelerometerZ)));
    float accY = atan(-1 * normalizedAccelerometerX / sqrt(sq(normalizedAccelerometerY) + sq(normalizedAccelerometerZ)));
    float accZ = atan2(a.acceleration.y, a.acceleration.x);
    unsigned long sampleMicros = (lastSampleMicros > 0) ? micros() - lastSampleMicros : 0;
    lastSampleMicros = micros();
    float gyroX = normalizedGyroscopeX * sampleMicros / 1000000;
    float gyroY = normalizedGyroscopeY * sampleMicros / 1000000;
    float gyroZ = normalizedGyroscopeZ * sampleMicros / 1000000;

    pitch = 0.98 * (pitch + degrees(gyroX)) + 0.02 * degrees(accX);
    roll = 0.98 * (roll + degrees(gyroY)) + 0.02 * degrees(accY);

    if (first) {
      neutralPitch = pitch;
      neutralRoll = roll;
      first = false;
    }

    myData.calibrationSuccessful = false;
    // calibrate
    if (buttonState == HIGH || calibrating) {
      delay(250);
      float pitchSum = 0;
      float rollSum = 0;
      for (int i = 0; i < 100; i++) {
        delay(30);
        mpu.getEvent(&a, &g, &temp);
        float normalizedGyroscopeX = g.gyro.x / 32.8;
        float normalizedGyroscopeY = g.gyro.z / 32.8;
        float normalizedGyroscopeZ = g.gyro.y / 32.8;
        float normalizedAccelerometerX = a.acceleration.x * 9.80665 / 16384;
        float normalizedAccelerometerY = a.acceleration.z * 9.80665 / 16384;
        float normalizedAccelerometerZ = a.acceleration.y * 9.80665 / 16384;

        float accX = atan(normalizedAccelerometerY / sqrt(sq(normalizedAccelerometerX) + sq(normalizedAccelerometerZ)));
        float accY = atan(-1 * normalizedAccelerometerX / sqrt(sq(normalizedAccelerometerY) + sq(normalizedAccelerometerZ)));
        float accZ = atan2(a.acceleration.y, a.acceleration.x);
        unsigned long sampleMicros = (lastSampleMicros > 0) ? micros() - lastSampleMicros : 0;
        lastSampleMicros = micros();
        float gyroX = normalizedGyroscopeX * sampleMicros / 1000000;
        float gyroY = normalizedGyroscopeY * sampleMicros / 1000000;
        float gyroZ = normalizedGyroscopeZ * sampleMicros / 1000000;

        pitch = 0.98 * (pitch + degrees(gyroX)) + 0.02 * degrees(accX);
        roll = 0.98 * (roll + degrees(gyroY)) + 0.02 * degrees(accY);

        pitchSum += pitch;
        rollSum += roll;
      };

      neutralPitch = pitchSum/100;
      neutralRoll = rollSum/100;
      Serial.println("Calibrating!!");
      myData.calibrationSuccessful = true;
    }

    myData.pitch = pitch;
    myData.roll = roll;
    myData.neutralPitch = neutralPitch;
    myData.neutralRoll = neutralRoll;
    myData.badPosture = false;
    //value to signify it comes from back and not knee
    myData.kneeAngle = -1;

    if (backOn) {
      if ((pitch - sensitivity > neutralPitch) || (pitch + sensitivity < neutralPitch) 
      || (roll - sensitivity > neutralRoll) || (roll + sensitivity < neutralRoll)) {
       
        Serial.print("pitch and rolls: ");
        Serial.print(pitch);
        Serial.print("  ");
        Serial.println(roll);

        if (timerDuration != 0) {
          myData.badPosture = true;
          if (timer.state() != RUNNING) {
            timer.start();
          }
        }
        
      } else {
        timer.stop();
        myservo.writePwm(motorPin, 0);
        Serial.println(" CORRECT POSE ");
      }
    }
    
    //send data to hub
    //make it send data less often, only when sendBuffer reaches a number
    if (sendBuffer == 200 || calibrating) {
      esp_err_t result = esp_now_send(hubAddress, (uint8_t *) &myData, sizeof(myData));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      sendBuffer = 0;
    } else {
      if (backOn) {
        sendBuffer++;
      }
    }

    //check timer
    if (timerDuration == 0) {
      timer.pause();
    } else if (timer.read() >= timerDuration) {
      timer.pause();
      if (vibMotorOn) {
        myservo.writePwm(motorPin, 155);
      }
    }

  }

  Serial.println("");
  delay(100);
}
