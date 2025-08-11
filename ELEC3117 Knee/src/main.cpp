#include <Arduino.h>
#include <cmath>

// Basic demo for accelerometer readings from Adafruit MPU6050
//MAC address for BACK f0:24:f9:e7:24:b0
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

outgoingBack myData;

esp_now_peer_info_t peerInfo;

const int flexPin = 35; 

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup(void) {
  
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


  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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

  delay(100);
}

void loop() {

  int sendBuffer = 0;
  int flexValue = analogRead(flexPin);
  Serial.print("sensor: ");
  Serial.println(flexValue);

  myData.kneeAngle = flexValue;
  esp_err_t result = esp_now_send(hubAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);

 
  delay(100);
}
