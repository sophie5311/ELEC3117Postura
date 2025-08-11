#include <Arduino.h>
#include "Timer.h"
#include <cmath>
#include <pwmWrite.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <FastLED.h>

#define BUTTON1 33
#define BUTTON2 32
#define BUTTON3 27
#define BUTTON4 26
#define BUZZER 16

#define NUM_LEDS 34
#define DATA_PIN 4
#define LED_TYPE WS2812B

#define HOME_PAGE 0
#define STATS_PAGE 1
#define CALIBRATE_PAGE 2
#define SETTINGS_PAGE_MAIN 3
#define HUB_SETTINGS 4
#define BACK_SETTINGS 5
#define KNEE_SETTINGS 6

#define STARTUP_SOUND 0
#define ALERT_SOUND 1
#define BREAK_SOUND 2

#define HIGHSEN 5
#define MEDSEN 10
#define LOWSEN 15

//Hub mac address
// f0, 24, f9, e7, 24, c8
//Back address f0:24:f9:e7:24:b0
uint8_t backAddress[] = {0xF0, 0x24, 0xF9, 0xE7, 0x24, 0xB0};

// uint8_t kneeAddress[] = {0xF0, 0x24, 0xF9, 0xE7, 0x24, 0xB0};

CRGB leds[NUM_LEDS];

//THE FIRST ARRAY ELEMENT CONTAINS THE SIZE OF THE ARRAY 
uint8_t neutralBack[] = {9, 6, 7, 12, 13, 19, 21, 25, 24, 23};
uint8_t backBack1[] = {9, 7, 8, 11, 12, 19, 21, 25, 24, 23};
uint8_t backBack2[] = {9, 8, 9, 10, 11, 20, 21, 25, 24, 23};
uint8_t forwardBack1[] = {9, 5, 6, 13, 14, 19, 21, 25, 24, 23};
uint8_t forwardBack2[] = {9, 4, 5, 14, 15, 18, 21, 23, 24, 25};
uint8_t forwardBack3[] = {8, 14, 15, 16, 17, 22, 25, 24, 23};

uint8_t stand[] = {11, 0, 1, 2, 3, 7, 12, 19, 21, 25, 26, 33};

uint8_t neutralLegs[] = {2, 28, 31};
uint8_t inLegs[] = {2, 27, 32};
uint8_t outLegs[] = {2, 29, 30};

int lcdColumns = 16;
int lcdRows = 2;

const int ledcChannel = 0;
const int resolution = 8;

byte start1x1[] = { B00000, B00000, B00000, B00000, B11111, B10010, B01010, B11010 };
byte start1x2[] = { B11000, B11100, B11000, B00000, B01011, B10110, B11111, B10110 };
byte start1x3[] = { B00000, B00000, B00000, B00000, B01110, B10100, B00100, B10100 };
byte stats1x5[] = { B00000, B00100, B00100, B00101, B00101, B00101, B00101, B00111 };
byte stats1x6[] = { B00000, B00000, B00000, B00000, B01000, B01000, B01010, B11111 };
byte cali1x9[] = { B00000, B00101, B00010, B00100, B00101, B00100, B00010, B00101 };
byte cali1x10[] = { B00000, B11010, B00100, B10010, B11010, B10010, B00100, B11010 };
byte sett1x13[] = { B00000, B00000, B00101, B00011, B00110, B00011, B00101, B00000 };
byte sett1x14[] = { B00000, B00000, B01011, B10000, B11000, B10000, B01011, B00000 };
byte sett1x15[] = { B00000, B01000, B11100, B01000, B00000, B10000, B11100, B10000 };
byte stop1x1[] = { B00000, B00001, B00001, B00000, B11111, B10010, B01010, B11010 };
byte stop1x2[] = { B00000, B01000, B01000, B00000, B01011, B10110, B10111, B01010 };
byte stop1x3[] = { B00000, B00000, B00000, B00000, B00000, B10000, B00000, B00000 };
byte backArrow1x0[] = { B00000, B00100, B01100, B11111, B01101, B00101, B00001, B01110 };
byte guyStand1x1[] = { B01100, B01100, B01000, B01000, B01100, B01010, B10010, B00000 };
byte Bell1x2[] = { B00000, B00100, B01110, B01110, B11111, B00100, B00000, B00000 };
byte speakerOn11x10[] = { B00000, B00001, B10111, B11111, B11111, B11111, B10111, B00001 };
byte speakerOn21x11[] = { B00000, B00100, B01000, B00000, B01110, B00000, B01000, B00100 };
byte LEDON11x7[] = { B01001, B00100, B00001, B00011, B11011, B00011, B00001, B00001 };
byte LEDON21x8[] = { B10010, B00100, B10000, B11000, B11011, B11000, B10000, B10000 };
byte LEDOFF11x7[] = { B00000, B00000, B00001, B00010, B00010, B00010, B00001, B00001 };
byte LEDOFF21x8[] = { B00000, B00000, B10000, B01000, B01000, B01000, B10000, B10000 };
byte slouchGuy1x1[] = { B00000, B00110, B00110, B01000, B10000, B11100, B00100, B00100 };
byte vibOn1x14[] = { B00000, B00100, B01000, B10011, B00111, B00100, B00111, B00001 };
byte vibOff1x14[] = { B00000, B00000, B00000, B00011, B00111, B00100, B00111, B00001 };
byte uprightGuy1x3[] = { B00000, B01100, B01100, B01000, B01000, B01110, B00010, B00010 };
byte tenx10[] = { B00000, B00000, B10111, B10101, B10101, B10111, B00000, B00000 };
byte last511x1[] = { B00000, B10010, B10101, B10111, B11101, B00000, B00000, B00000 };
byte last521x2[] = { B00000, B11111, B10010, B01010, B11010, B00000, B00000, B00000 };
byte last531x3[] = { B00000, B11100, B00101, B11100, B00101, B11100, B00000, B00000 };


int yellow[] = {20, 30, 0};
int orange[] = {40, 10, 0};
int red[] = {50, 0, 0};
int blue[] = {0, 0, 50};
int green[] = {0, 50, 0};
int purple[] = {25, 25, 0};

// SETTINGS AND STATUS
int page;
int prevPage;
bool updatePage = false;
bool sessionStarted = false;
bool backConnected = false;
bool kneeConnected = false;
bool ledsOn = true;
unsigned long breakTimer = 1200000;
bool soundOn = true;

float timerBack = 2000;
bool vibrationMotorOn = true;
float sensitivity = 5;

bool calibrating = false;

float pitchRec = 0;
float rollRec = 0;
float neutralPitchRec = 0;
float neutralRollRec = 0;

float kneeAngle;

unsigned long breakTimeStart;

int threeSeshAvg = 111;
int threeAgoSesh = 111;
int twoAgoSesh = 111;
int oneAgoSesh = 111;

int currSeshUprightSec = 0;
int currSeshSlouchSec = 0;
long postureBadTime = 0;
long postureGoodTime = 0;


// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

Preferences preferences;

Timer timer1;

typedef struct incomingBack {
  float pitch;
  float roll;
  bool badPosture;
  float neutralPitch;
  float neutralRoll;
  bool calibrationSuccessful;
  float kneeAngle;
} incomingBack;

// typedef struct incomingKnee {
//   float kneeAngle;
// } incomingKnee;

typedef struct outgoingHub {
  bool back;
  bool knee;
  float timer;
  bool calibrate;
  bool vibMotor;
  float sensitivity;
} outgoingHub;

incomingBack backData;
// incomingKnee kneeData;
outgoingHub myData;

esp_now_peer_info_t peerInfo;

void updateLEDS(float pitch, float roll, float neutralPitch, float neutralRoll);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&backData, incomingData, sizeof(backData));
  if (backData.kneeAngle == -1) {
    if (backData.calibrationSuccessful) {
      calibrating = false;
    }
    Serial.print("Recieved Data");

    if (backData.badPosture) {
      if (timer1.state() != RUNNING) {
        timer1.start();
      }
    } else {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(1, 0);
      lcd.print("Good posture");
      timer1.pause();
      if (postureGoodTime == -1) {
        currSeshSlouchSec += millis() - postureBadTime;
        postureBadTime = -1;
        postureGoodTime = millis();
      }
    }

    if (backConnected) {
      pitchRec = backData.pitch;
      rollRec = backData.roll;
      neutralPitchRec = backData.neutralPitch;
      neutralRollRec = backData.neutralRoll;
      updateLEDS(backData.pitch, backData.roll, backData.neutralPitch, backData.neutralRoll);
    }

  } else {
    //message from knee module
  }
    
  

}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Sent data (dont know if successful)");
  if (status != 0) {
    Serial.print("Connection failed");
  }
}

void playSound(int sound) {
  if (sound == STARTUP_SOUND) {
    ledcWriteTone(ledcChannel, 494);
    delay(150);
    ledcWriteTone(ledcChannel, 659);
    delay(300);
    ledcWriteTone(ledcChannel, 740);
    delay(150);
    ledcWriteTone(ledcChannel, 988);
    delay(500);
    ledcWriteTone(ledcChannel, 0);
  } else if (sound == ALERT_SOUND) {
    ledcWriteTone(ledcChannel, 660);
    delay(150);
    ledcWriteTone(ledcChannel, 622);
    delay(150);
    ledcWriteTone(ledcChannel, 660);
    delay(150);
    ledcWriteTone(ledcChannel, 0);
  } else if (sound == BREAK_SOUND) {
    ledcWriteTone(ledcChannel, 494);
    delay(200);
    ledcWriteTone(ledcChannel, 622);
    delay(200);
    ledcWriteTone(ledcChannel, 740);
    delay(200);
    ledcWriteTone(ledcChannel, 988);
    delay(400);
    ledcWriteTone(ledcChannel, 0);
  }
}

void clearLEDS(bool NOW) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
  if (NOW) {
    FastLED.show();
  }
}

void setLEDs(uint8_t ledPattern[], int colours[3]) {
  int size = ledPattern[0];
  for (int i = 1; i <= size; i++) {
    uint8_t ledPos = ledPattern[i];
    leds[ledPos].r = colours[0];
    leds[ledPos].g = colours[1];
    leds[ledPos].b = colours[2];
  }

  FastLED.show();
}

void updateLEDS(float pitch, float roll, float neutralPitch, float neutralRoll) {
  if (ledsOn) {
    clearLEDS(false);
    int *colour;
    Serial.print("pitch and rolls:");
    Serial.print(pitch);
    Serial.print("   ");
    Serial.println(roll);
    Serial.print("neutral pitch and rolls:");
    Serial.print(neutralPitch);
    Serial.print("   ");
    Serial.println(neutralRoll);
    if (backConnected) {
      if (pitch > neutralPitch + sensitivity && pitch <= neutralPitch + 2*sensitivity) {
        //back 1
        setLEDs(backBack1, orange);
        colour = orange;
        Serial.print("back1");
      } else if (pitch > neutralPitch + 2*sensitivity) {
        //back 2
        setLEDs(backBack2, red);
        colour = red;
        Serial.print("back2");
      } else if (pitch < neutralPitch - sensitivity && pitch >= neutralPitch - 2*sensitivity) {
        //back 1
        Serial.print("forward1");
        setLEDs(forwardBack1, yellow);
        colour = yellow;
      } else if (pitch < neutralPitch - 2*sensitivity && pitch >= neutralPitch - 4*sensitivity) {
        //forward 2
        Serial.print("forward2");
        setLEDs(forwardBack2, orange);
        colour = orange;
      } else if (pitch < neutralPitch - 4*sensitivity) {
        //forward 3
        Serial.print("forward3");
        setLEDs(forwardBack3, red);
        colour = red;
      } else {
        setLEDs(neutralBack, green);
        colour = green;
      }
    } else {
      setLEDs(neutralBack, blue);
      colour = blue;
      Serial.print("neutral");
    }

    if (kneeConnected) {
      // KNEE STUFF
      setLEDs(neutralLegs, colour);
    } else {
      setLEDs(neutralLegs, colour);
    }
    
  } else {
    clearLEDS(true);
  }
}

void setupStartPage() {
  prevPage = HOME_PAGE;
  page = HOME_PAGE;
  sessionStarted = false;
  clearLEDS(false);
  setLEDs(neutralBack, blue);
  setLEDs(neutralLegs, blue);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Postura");

  lcd.createChar(0, start1x1);
  lcd.setCursor(1, 1);
  lcd.write(0);
  
  lcd.createChar(1, start1x2);
  lcd.setCursor(2, 1);
  lcd.write(1);
  
  lcd.createChar(2, start1x3);
  lcd.setCursor(3, 1);
  lcd.write(2);
  
  lcd.createChar(3, stats1x5);
  lcd.setCursor(5, 1);
  lcd.write(3);
  
  lcd.createChar(4, stats1x6);
  lcd.setCursor(6, 1);
  lcd.write(4);
  
  lcd.setCursor(9, 1);
  lcd.print("cal");
  
  lcd.createChar(5, sett1x13);
  lcd.setCursor(13, 1);
  lcd.write(5);
  
  lcd.createChar(6, sett1x14);
  lcd.setCursor(14, 1);
  lcd.write(6);
  
  lcd.createChar(7, sett1x15);
  lcd.setCursor(15, 1);
  lcd.write(7);

  playSound(STARTUP_SOUND);
}

void sendData() {
  myData.back = backConnected;
  myData.knee = kneeConnected;
  myData.timer = timerBack;
  myData.calibrate = calibrating;
  myData.sensitivity = sensitivity;
  myData.vibMotor = vibrationMotorOn;
  esp_err_t result = esp_now_send(backAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
    

uint8_t buttonState1Debounce = 0b00000001;
uint8_t buttonState2Debounce = 0b00000001;
uint8_t buttonState3Debounce = 0b00000001;
uint8_t buttonState4Debounce = 0b00000001;
int buttonState1DebouncePrev = LOW;
int buttonState2DebouncePrev = LOW;
int buttonState3DebouncePrev = LOW;
int buttonState4DebouncePrev = LOW;

void updateButtons() {
  int buttonState1 = digitalRead(BUTTON1);
  if (buttonState1 == HIGH) {
    buttonState1Debounce <<= 1;
  } else {
    buttonState1DebouncePrev == LOW;
    buttonState1Debounce = 0b00000001;
  }
  
  int buttonState2 = digitalRead(BUTTON2);
  if (buttonState2 == HIGH) {
    buttonState2Debounce <<= 1;
  } else {
    buttonState2DebouncePrev == LOW;
    buttonState2Debounce = 0b00000001;
  }

  int buttonState3 = digitalRead(BUTTON3);
  if (buttonState3 == HIGH) {
    buttonState3Debounce <<= 1;
  } else {
    buttonState3DebouncePrev == LOW;
    buttonState3Debounce = 0b00000001;
  }

  int buttonState4 = digitalRead(BUTTON4);
  if (buttonState4 == HIGH) {
    buttonState4Debounce <<= 1;
  } else {
    buttonState4DebouncePrev == LOW;
    buttonState4Debounce = 0b00000001;
  }
}


void drawStartStop(bool onHome) {
  if ((sessionStarted == false && onHome) || (sessionStarted == true && !onHome)) {
    Serial.println("STARTING SESSION");
    lcd.createChar(0, stop1x1);
    lcd.setCursor(1, 1);
    lcd.write(0);

    lcd.createChar(1, stop1x2);
    lcd.setCursor(2, 1);
    lcd.write(1);

    lcd.createChar(2, stop1x3);
    lcd.setCursor(3, 1);
    lcd.write(2);

    if (onHome) {
      clearLEDS(false);
      setLEDs(neutralBack, green);
      setLEDs(neutralLegs, green);
    } else {
      clearLEDS(false);
      setLEDs(neutralBack, blue);
      setLEDs(neutralLegs, blue);
    }
  } else if ((sessionStarted == true && onHome) || (sessionStarted == false && !onHome)) {
    Serial.println("ENDING SESSION");
    lcd.createChar(0, start1x1);
    lcd.setCursor(1, 1);
    lcd.write(0);
    
    lcd.createChar(1, start1x2);
    lcd.setCursor(2, 1);
    lcd.write(1);
    
    lcd.createChar(2, start1x3);
    lcd.setCursor(3, 1);
    lcd.write(2);

    if (onHome) {
      clearLEDS(false);
      setLEDs(neutralBack, blue);
      setLEDs(neutralLegs, blue);
    } 
  }
}

void drawHomePage() {
  page = HOME_PAGE;
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Postura");

  drawStartStop(false);
  
  lcd.createChar(3, stats1x5);
  lcd.setCursor(5, 1);
  lcd.write(3);
  
  lcd.createChar(4, stats1x6);
  lcd.setCursor(6, 1);
  lcd.write(4);
  
  lcd.setCursor(9, 1);
  lcd.print("cal");
  
  lcd.createChar(5, sett1x13);
  lcd.setCursor(13, 1);
  lcd.write(5);
  
  lcd.createChar(6, sett1x14);
  lcd.setCursor(14, 1);
  lcd.write(6);
  
  lcd.createChar(7, sett1x15);
  lcd.setCursor(15, 1);
  lcd.write(7);
}

bool calibrate() {
  lcd.clear();
  lcd.print("Sit comfortably");
  lcd.setCursor(2, 1);
  lcd.print("Calibrating");

  calibrating = true;
  sendData();

  int attempt = 0;
  Serial.print(calibrating);
  while (calibrating == true) {
    Serial.print(calibrating);
    lcd.setCursor(13, 1);
    lcd.print(".");
    delay(250);
    lcd.setCursor(14, 1);
    lcd.print(".");
    delay(250);
    lcd.setCursor(15, 1);
    lcd.print(".");
    delay(250);
    lcd.setCursor(13, 1);
    lcd.print("   ");
    delay(250);
    attempt++;

    if (attempt >= 5) {
      lcd.clear();
      lcd.print("Calibration");
      lcd.setCursor(0, 1);
      lcd.print("Failed :(");
      delay(2000);
      Serial.print("CalibrationFailed");
      page = HOME_PAGE;
      drawHomePage();
      return false;
    }
  }

  Serial.println("calibrating successful in func");
  lcd.clear();
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Successful!");
  delay(2000);
  page = HOME_PAGE;
  drawHomePage();
  return true;
}

void updateSessionData(int latestSessionData) {
  threeAgoSesh = twoAgoSesh;
  twoAgoSesh = oneAgoSesh;
  oneAgoSesh = latestSessionData;
  
  if (threeAgoSesh != 111 && twoAgoSesh != 111 && oneAgoSesh != 111) {
    threeSeshAvg = (threeAgoSesh + twoAgoSesh + oneAgoSesh)/3;
    preferences.putInt("threeSeshAvg", threeSeshAvg);
  }

  //LONG TERM FLASH
  if (threeAgoSesh != 111) {
    preferences.putInt("threeAgoSesh", threeAgoSesh);
  }
  if (twoAgoSesh != 111) {
    preferences.putInt("twoAgoSesh", twoAgoSesh);
  }
  if (oneAgoSesh != 111) {
    preferences.putInt("oneAgoSesh", oneAgoSesh);
  }

}

void startStopSession() {
  if (sessionStarted == false) {
    //start session
    backConnected = true;
    kneeConnected = true;

    if (calibrate()) {
      Serial.print("CalibrationSucessful");
      drawStartStop(true);
      sessionStarted = true;
      if (breakTimer != 0) {
        breakTimeStart = millis();
        Serial.print("break time start is");
        Serial.println(breakTimeStart);
      }
      postureGoodTime = millis();
      postureBadTime = -1;
      currSeshSlouchSec = 0;
      currSeshUprightSec = 0;
    } else {
      Serial.print("CalibrationFailed");
    }

  } else {
    drawStartStop(true);
    backConnected = false;
    kneeConnected = false;

    lcd.setCursor(0, 0);
    lcd.print("              ");
    lcd.setCursor(5, 0);
    lcd.print("Postura");
    sessionStarted = false;

    Serial.println("Session Ended");
    Serial.print("numer: ");
    Serial.println(currSeshUprightSec*100);
    Serial.print("denom: ");
    Serial.print(currSeshSlouchSec);
    Serial.print("     ");
    Serial.print(currSeshUprightSec);
    Serial.print("     ");
    Serial.print((currSeshSlouchSec + currSeshUprightSec));
    if (currSeshSlouchSec + currSeshUprightSec != 0) {
      updateSessionData((int)(currSeshUprightSec*100/(currSeshSlouchSec + currSeshUprightSec)));
    } else {
      Serial.print("ERROR: demon is zero. :(");
    }
    

    timer1.pause();
  }
  
}

void setup(){
  // Initialize Serial Monitor
  delay(1000);
  Serial.begin(115200);
  
  //set up leds
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, backAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //Setting up gpio pins
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  pinMode(BUTTON4, INPUT);
  ledcSetup(ledcChannel, 2000, resolution);
  ledcAttachPin(BUZZER, ledcChannel);

  // Set state to inital state 
  setupStartPage();

  lcd.noAutoscroll();
  lcd.noCursor();
  lcd.noBlink();

  //LONG TERM FLASH
  preferences.begin("data", false); 
  threeAgoSesh = preferences.getInt("threeAgoSesh", 111);
  twoAgoSesh = preferences.getInt("twoAgoSesh", 111);
  oneAgoSesh = preferences.getInt("oneAgoSesh", 111);
  threeSeshAvg = preferences.getInt("threeSeshAvg", 111);
  

}

void loop(){
  bool buttonPressed = false;
  //UI FUNCTIONS
  updateButtons();
  
  //HOME PAGE
  if (page == HOME_PAGE) {
    //BUTTON 1 IS PRESSED
    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      Serial.println("triggered!!");
      buttonState1DebouncePrev == HIGH;

      startStopSession();      
      buttonPressed = true;  
    }

    //BUTTON 2 IS PRESSED
    if ((buttonState2Debounce == 0b10000000) && (buttonState2DebouncePrev == LOW)) {
      buttonState2DebouncePrev == HIGH;
      page = STATS_PAGE;
      lcd.clear();
      lcd.print(" sesh avg  :   %");
      lcd.createChar(0, uprightGuy1x3);
      lcd.setCursor(10, 0);
      lcd.write(0);

      lcd.setCursor(0, 0);
      lcd.print("3");
      
      if (threeSeshAvg != 111) {
        lcd.setCursor(12, 0);
        lcd.print(threeSeshAvg);
      }
    
      lcd.createChar(1, backArrow1x0);
      lcd.setCursor(0, 1);
      lcd.write(1);
      buttonPressed = true;   

      lcd.createChar(3, last511x1);
      lcd.setCursor(1, 1);
      lcd.write(3);

      lcd.createChar(4, last521x2);
      lcd.setCursor(2, 1);
      lcd.write(4);

      lcd.createChar(5, last531x3);
      lcd.setCursor(3, 1);
      lcd.write(5);

      if (threeAgoSesh != 111) {
        lcd.setCursor(5, 1);
        lcd.print(threeAgoSesh);
      }

      if (twoAgoSesh != 111) {
        lcd.setCursor(9, 1);
        lcd.print(twoAgoSesh);
      }

      if (oneAgoSesh != 111) {
        lcd.setCursor(13, 1);
        lcd.print(oneAgoSesh);
      }
      
    }

    //BUTTON 3 IS PRESSED
    if ((buttonState3Debounce == 0b10000000) && (buttonState3DebouncePrev == LOW)) {
      buttonState3DebouncePrev == HIGH;
      page = CALIBRATE_PAGE;
      calibrate();
      buttonPressed = true;   
    }

    //BUTTON 4 IS PRESSED
    if ((buttonState4Debounce == 0b10000000) && (buttonState4DebouncePrev == LOW)) {
      buttonState4DebouncePrev == HIGH;
      page = SETTINGS_PAGE_MAIN;
      lcd.clear();
      lcd.print("SETTINGS");

      lcd.createChar(1, backArrow1x0);
      lcd.setCursor(0, 1);
      lcd.write(1);

      lcd.setCursor(2, 1);
      lcd.print("Hub");

      lcd.setCursor(6, 1);
      lcd.print("Back");

      lcd.setCursor(11, 1);
      lcd.print("Knee");

      buttonPressed = true;   
    }
  } else if (page == STATS_PAGE) {
    Serial.print("ON STATS PAGE");
    //BUTTON 1 IS PRESSED
    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      buttonState1DebouncePrev == HIGH;
      drawHomePage();
      buttonPressed = true;   
    }

  } else if (page == SETTINGS_PAGE_MAIN) {
    //BUTTON 1 IS PRESSED
    Serial.print("ON SETT MAIN PAGE");
    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      buttonState1DebouncePrev == HIGH;
      drawHomePage();
      buttonPressed = true;   
    }

    //BUTTON 2 IS PRESSED
    if ((buttonState2Debounce == 0b10000000) && (buttonState2DebouncePrev == LOW)) {
      buttonState2DebouncePrev == HIGH;
      page = HUB_SETTINGS;
      prevPage = SETTINGS_PAGE_MAIN;
      buttonPressed = true;   
    }

    //BUTTON 3 IS PRESSED
    if ((buttonState3Debounce == 0b10000000) && (buttonState3DebouncePrev == LOW)) {
      buttonState3DebouncePrev == HIGH;
      //back settings
      page = BACK_SETTINGS;
      prevPage = SETTINGS_PAGE_MAIN;
      buttonPressed = true;   
    }

    // BUTTON 4 IS PRESSED
    if ((buttonState4Debounce == 0b10000000) && (buttonState4DebouncePrev == LOW)) {
      buttonState4DebouncePrev == HIGH;
      //knee settings
      page = KNEE_SETTINGS;
      prevPage = SETTINGS_PAGE_MAIN;
      buttonPressed = true;
    }

  } else if (page == HUB_SETTINGS) {
    //BUTTON 1 IS PRESSED
    if (prevPage != page || updatePage == true) {
      Serial.print("ON HUB SETT PAGE");
      lcd.clear();
      lcd.createChar(1, backArrow1x0);
      lcd.setCursor(0, 1);
      lcd.write(1);

      lcd.setCursor(0, 0);
      lcd.print("Hub Settings");

      lcd.createChar(0, guyStand1x1);
      lcd.setCursor(4, 1);
      lcd.write(0);
      
      lcd.createChar(2, Bell1x2);
      lcd.setCursor(5, 1);
      lcd.write(2);

      if (breakTimer == 180000) {
        //3mins
        lcd.setCursor(6, 1);
        lcd.print("3m");
      } else if (breakTimer == 1200000) {
        //20 mins
        lcd.setCursor(6, 1);
        lcd.print("20m");
      } else if (breakTimer == 2400000) {
        //40 mins
        lcd.setCursor(6, 1);
        lcd.print("40m");
      } else if (breakTimer == 3600000) {
        //1 hr
        lcd.setCursor(6, 1);
        lcd.print("60m");
      } else if (breakTimer == 0) {
        //OFF
        lcd.setCursor(6, 1);
        lcd.print("Off");
      }

      if (ledsOn) {
        lcd.createChar(5, LEDON11x7);
        lcd.setCursor(10, 1);
        lcd.write(5);

        lcd.createChar(6, LEDON21x8);
        lcd.setCursor(11, 1);
        lcd.write(6);
      } else {
        lcd.createChar(5, LEDOFF11x7);
        lcd.setCursor(10, 1);
        lcd.write(5);

        lcd.createChar(6, LEDOFF21x8);
        lcd.setCursor(11, 1);
        lcd.write(6);
      }

      lcd.createChar(3, speakerOn11x10);
      lcd.setCursor(13, 1);
      lcd.write(3);
      if (soundOn) {
        lcd.createChar(4, speakerOn21x11);
        lcd.setCursor(14, 1);
        lcd.write(4);
      }
      updatePage = false;
      prevPage = page;
    }
    

    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      buttonState1DebouncePrev == HIGH;
      //back
      drawHomePage();
      buttonPressed = true;   
    }

    //BUTTON 2 IS PRESSED
    if ((buttonState2Debounce == 0b10000000) && (buttonState2DebouncePrev == LOW)) {
      buttonState2DebouncePrev == HIGH;
      //break
      if (breakTimer == 180000) {
        breakTimer = 1200000;
      } else if (breakTimer == 1200000) {
        breakTimer = 2400000;
      } else if (breakTimer == 2400000) {
        breakTimer = 3600000;
      } else if (breakTimer == 3600000) {
        breakTimer = 0;
      } else if (breakTimer == 0) {
        breakTimer = 180000;
      }
      updatePage = true;
      buttonPressed = true;   
    }

    //BUTTON 3 IS PRESSED
    if ((buttonState3Debounce == 0b10000000) && (buttonState3DebouncePrev == LOW)) {
      buttonState3DebouncePrev == HIGH;
      //leds
      if (ledsOn) {
        ledsOn = false;
        clearLEDS(true);
        // delay(5000);
      } else {
        ledsOn = true;
        updateLEDS(pitchRec, rollRec, neutralPitchRec, neutralRollRec);
      }
      updatePage = true;
      buttonPressed = true;   
    }

    //BUTTON 4 IS PRESSED
    if ((buttonState4Debounce == 0b10000000) && (buttonState4DebouncePrev == LOW)) {
      buttonState4DebouncePrev == HIGH;
      //sounds
      if (soundOn) {
        soundOn = false;
      } else {
        soundOn = true;
      }

      updatePage = true;
      buttonPressed = true;   
    }

  } else if (page == BACK_SETTINGS) {
    //BUTTON 1 IS PRESSED
    if (prevPage != page || updatePage == true) {
      Serial.print("ON BACK SETT PAGE");
      lcd.clear();
      lcd.createChar(1, backArrow1x0);
      lcd.setCursor(0, 1);
      lcd.write(1);

      lcd.setCursor(0, 0);
      lcd.print("Back Settings");

      lcd.createChar(0, slouchGuy1x1);
      lcd.setCursor(2, 1);
      lcd.write(0);
      
      lcd.createChar(2, Bell1x2);
      lcd.setCursor(3, 1);
      lcd.write(2);

      if (timerBack == 2000) {
        //2s
        lcd.setCursor(4, 1);
        lcd.print("2s");
      } else if (timerBack == 5000) {
        //5s
        lcd.setCursor(4, 1);
        lcd.print("5s");
      } else if (timerBack == 10000) {
        //10s
        lcd.setCursor(4, 1);
        lcd.print("10s");
      } else if (timerBack == 0) {
        //OFF
        lcd.setCursor(4, 1);
        lcd.print("Off");
      }

      lcd.setCursor(8, 1);
      lcd.print("SEN");
      if (sensitivity == HIGHSEN) {
        lcd.setCursor(11, 1);
        lcd.print("HI");
      } else if (sensitivity == MEDSEN) {
        lcd.setCursor(11, 1);
        lcd.print("ME");
      } else if (sensitivity == LOWSEN) {
        lcd.setCursor(11, 1);
        lcd.print("LO");
      }
      
      if (vibrationMotorOn) {
        lcd.createChar(5, vibOn1x14);
        lcd.setCursor(14, 1);
        lcd.write(5);
      } else {
        lcd.createChar(5, vibOff1x14);
        lcd.setCursor(14, 1);
        lcd.write(5);
      }
      updatePage = false;
      prevPage = page;
    }

    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      buttonState1DebouncePrev == HIGH;
      //back
      drawHomePage();
      buttonPressed = true;   
    }

    //BUTTON 2 IS PRESSED
    if ((buttonState2Debounce == 0b10000000) && (buttonState2DebouncePrev == LOW)) {
      buttonState2DebouncePrev == HIGH;
      //timer
      if (timerBack == 2000) {
        timerBack = 5000;
      } else if (timerBack == 5000) {
        timerBack = 10000;
      } else if (timerBack == 10000) {
        timerBack = 0;
      } else if (timerBack == 0) {
        timerBack = 2000;
      }
      updatePage = true;
      buttonPressed = true;   
    }

    //BUTTON 3 IS PRESSED
    if ((buttonState3Debounce == 0b10000000) && (buttonState3DebouncePrev == LOW)) {
      buttonState3DebouncePrev == HIGH;
      //
      //timer
      if (sensitivity == HIGHSEN) {
        sensitivity = MEDSEN;
      } else if (sensitivity == MEDSEN) {
        sensitivity = LOWSEN;
      } else if (sensitivity == LOWSEN) {
        sensitivity = HIGHSEN;
      }
      updatePage = true;
      buttonPressed = true;   
    }

    //BUTTON 4 IS PRESSED
    if ((buttonState4Debounce == 0b10000000) && (buttonState4DebouncePrev == LOW)) {
      buttonState4DebouncePrev == HIGH;
      if (vibrationMotorOn) {
        vibrationMotorOn = false;
      } else {
        vibrationMotorOn = true;
      }
      updatePage = true;
      buttonPressed = true;   
    }

  } else if (page == KNEE_SETTINGS) {
    if (prevPage != page || buttonPressed == true) {
      Serial.print("ON KNEE SETT PAGE");
      //BUTTON 1 IS PRESSED
      lcd.clear();
      lcd.createChar(1, backArrow1x0);
      lcd.setCursor(0, 1);
      lcd.write(1);

      lcd.setCursor(0, 0);
      lcd.print("KNEE SETTINGS");
      updatePage = false;
      prevPage = page;
    }

    if ((buttonState1Debounce == 0b10000000) && (buttonState1DebouncePrev == LOW)) {
      buttonState1DebouncePrev == HIGH;
      //back
      drawHomePage();
      buttonPressed = true;   
    }

  }


  if (timerBack != 0 && timer1.read() >= timerBack && backConnected) {
    timer1.pause();
    if (soundOn) {
      playSound(ALERT_SOUND);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("BAD POSTURE!");
    
    if (postureBadTime == -1) {
      Serial.print("Posture good time: ");
      // Serial.print(postureGoodTime);
      currSeshUprightSec += millis() - postureGoodTime;
      postureBadTime = millis();
      postureGoodTime = -1;
    }
    
  }

  
  if (breakTimer != 0 && sessionStarted == true) {
    
    if ((millis() - breakTimeStart) > breakTimer) {
      Serial.print("TIme for break!");
      if (soundOn) {
        playSound(BREAK_SOUND);
      }
      startStopSession();
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print("Stand for break!");
      delay(5000);
      lcd.print("                ");
      lcd.setCursor(5, 0);
      lcd.print("Postura");
      breakTimeStart = 0;
    }
  }

  if (buttonPressed == true) {
    buttonPressed = false;
    sendData();
  }
}

