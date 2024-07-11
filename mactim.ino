#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Function declarations
void checkAccelerometer();
void checkVibration();
void displayString(String str, int row, int col);
void sendSMS(String recipient, String message);
String getGoogleMapsLink();
float calculateSpeed();
void displaySpeed(float speed);
void countPulse();
void checkSpeed();

// Pin definitions
#define RED_BUTTON_PIN 2
#define WHITE_BUTTON_PIN 3
#define irPin 4
#define accInterruptPin 5
#define vibrationSensorPin 6
#define buzzerPin 7
#define GSM_RX_PIN 10
#define GSM_TX_PIN 11
#define GPS_RX_PIN 14
#define GPS_TX_PIN 15

const float tireRadius = 0.3; // Radius of the tire in meters
const float tireCircumference = 2 * PI * tireRadius;
const int numSpokes = 6;
const int speedLimit = 30;

volatile unsigned int pulseCount = 0; // Counter for the number of pulses from the IR sensor
unsigned long prevTime = 0; // Previous time of pulse count measurement

const int MPU = 0x68; // MPU6050 I2C address

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

bool antiTheftMode = false;
double latitude, longitude;
String lat_str, lng_str;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(vibrationSensorPin, INPUT);
  pinMode(WHITE_BUTTON_PIN, INPUT);
  pinMode(RED_BUTTON_PIN, INPUT);
  pinMode(accInterruptPin, INPUT);
  pinMode(GSM_RX_PIN, INPUT);
  pinMode(GSM_TX_PIN, OUTPUT);

  Serial.begin(9600);
  Wire.begin();                      // Initialize communication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // End the transmission

  gsmSerial.begin(9600);
  // Configure SIM900A module
  delay(200);
  gsmSerial.println("AT");
  delay(200);
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(200);

  gpsSerial.begin(19200);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MaCTiM");
  delay(1000);
  lcd.setCursor(1, 0);
  lcd.print("Power ON");
  delay(2000);
  lcd.clear();
}

void loop() {
  if (digitalRead(WHITE_BUTTON_PIN) == HIGH) {
    delay(50);  // Debounce delay
    if (digitalRead(WHITE_BUTTON_PIN) == LOW) {
      antiTheftMode = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Anti-theft on");
      delay(2000);
    }
  } else if (digitalRead(RED_BUTTON_PIN) == HIGH) {
    delay(50);  // Debounce delay
    if (digitalRead(RED_BUTTON_PIN) == LOW) {
      antiTheftMode = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Anti-theft off");
    }
  }

  // Additional code for anti-theft mode
  if (antiTheftMode) {
    checkVibration();
    checkAccelerometer();
  } else {
    checkSpeed();
  }
}

void checkVibration() {
  if (digitalRead(vibrationSensorPin) == HIGH) {
    digitalWrite(buzzerPin, LOW);
  } else {
    displayString("ALERT!!", 0, 0);
    delay(50);
    displayString("Vibration Detected", 1, 0);
    delay(50);
    sendSMS("+9475-------", "ALERT! Vibration detected. Bike Location: " + getGoogleMapsLink());
    delay(50);
    digitalWrite(buzzerPin, HIGH);
    delay(2000);
    digitalWrite(buzzerPin, LOW);
    delay(2000);
    lcd.clear();
  }
}

void checkAccelerometer() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float roll = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(ax, az) * 180.0 / PI;

  if (abs(roll) > 60 || abs(pitch) > 60) {
    displayString("ALERT!!", 0, 0);
    delay(50);
    displayString("Bike Tilted", 1, 0);
    delay(50);
    sendSMS("+9475-------", "ALERT! Bike tilted. Bike Location: " + getGoogleMapsLink());
    digitalWrite(buzzerPin, HIGH);
    delay(2000);
    digitalWrite(buzzerPin, LOW);
    delay(2000);
    lcd.clear();
  }
}

void displayString(String str, int row, int col) {
  lcd.setCursor(col, row);
  lcd.print(str);
}

void sendSMS(String recipient, String message) {
  gsmSerial.println("AT+CMGF=1");
  delay(100);

  gsmSerial.println("AT+CMGS=\"" + recipient + "\"");
  delay(100);

  gsmSerial.println(message);
  delay(100);

  gsmSerial.write(26);
  delay(100);

  Serial.println("SMS Sent successfully!");
}

void checkSpeed() {
  if (millis() - prevTime >= 10000) {
    float speed = calculateSpeed();
    displaySpeed(speed);
    prevTime = millis();
  }
}

void countPulse() {
  pulseCount++;
}

float calculateSpeed() {
  float distance = (pulseCount * tireCircumference) / 1000.0; // Convert to kilometers
  float speed = (distance / 1.0) * 3.6; // Convert to km/h
  pulseCount = 0; // Reset the pulse count
  return speed;
}

void displaySpeed(float speed) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(speed, 1);
  lcd.print(" km/h");

  if (speed >= speedLimit) {
    lcd.setCursor(0, 1);
    lcd.print("Speed limit exceeded!");
  }
}

String getGoogleMapsLink() {
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        lat_str = String(latitude, 5);
        longitude = gps.location.lng();
        lng_str = String(longitude, 5);
      }
      Serial.println(lng_str);
      Serial.println(lat_str);
    }
  delay(100);
  String googleMapsLink = "https://www.google.com/maps/place/";
  googleMapsLink += lat_str;
  googleMapsLink += ",";
  googleMapsLink += lng_str;

  return googleMapsLink;
}
