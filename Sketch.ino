#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "DHTesp.h"
#include "ThingSpeak.h"

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x3F, 16 column and 2 rows

#define TRIG_PIN 18 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 5 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin
#define LED 2  //LED pin position; LED resistances are set at 220
#define LED2 4
#define LED3 15
#define LED4 19
#define updateTime 50  // increase this delay for they system to run faster or slower
float duration_us, distance_cm, last_distance;
const int servoPin = 13;  // pin servo is connected to

// esp32 wifi parameters
const char* WIFI_NAME = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const int myChannelNumber = YourChannelNumber; // remove YourChannelNumber and add thingspeak channel ID
const char* myApiKey = "YourWriteAPI"; //write API key from ThingSpeak
const char* server = "api.thingspeak.com";

WiFiClient client;
Servo servo;

void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  lcd.init();  // initialize the lcd
  lcd.backlight();  // open the backlight
  servo.attach(servoPin, 500, 2400);

  //esp32 module code
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Wifi not connected");
  }
  Serial.println("Wifi connected !");
  Serial.println("------");
  Serial.println("Local IP: " + String(WiFi.localIP()));
  Serial.println("------");
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client); 
}

int pos = 0; // Initialize servo position

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // to 180 incrementing
    servo.write(pos);
    ThingSpeak.setField(1, distance_cm);
    ThingSpeak.setField(2, pos);

    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    
    // calculate the distance
    // Need to Figure out why this works?
    distance_cm = 0.017 * duration_us;
  
    // light up led if distance under limit
    if (distance_cm < 30) {digitalWrite(LED, HIGH);
    }else{digitalWrite(LED, LOW);
    }
    if (distance_cm < 200) {digitalWrite(LED2, HIGH);
    }else{digitalWrite(LED2, LOW);
    }
    if (distance_cm < 399) {digitalWrite(LED3, HIGH); // set to 399 so changes can be seen
    }else{digitalWrite(LED3, LOW);
    }

    if (distance_cm != last_distance) {; // if distance is the same dont re-print it to lcd
    lcd.setCursor(0, 0); // start to print at the first row
    lcd.print("Distance: ");
    lcd.print(distance_cm);
    lcd.print(" cm");
    }
    lcd.setCursor(0, 1); // point at second row
    lcd.print("Angle: ");
    lcd.println(pos);
    lcd.print("Degrees"); // ° symbol cannot print
    //delay(445);

    // print the value to Serial Monitor
    if (distance_cm != last_distance) {;
    Serial.print("distanceA: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    Serial.print("at Angle: ");
    Serial.print(pos);
    Serial.println(" degrees");
    Blink(LED4);
    last_distance = distance_cm;
    int x = ThingSpeak.writeFields(myChannelNumber,myApiKey);  // from example
    if(x == 200){
    Serial.println("Data pushed successfull");
    }else{
    Serial.println("Push error" + String(x));
    }
    Serial.println("-------");
    delay(updateTime);
    }
  }

  for (pos = 180; pos >= 0; pos -= 1) { // from 180 decrementing
    ThingSpeak.setField(1, distance_cm);
    ThingSpeak.setField(2, pos);
    servo.write(pos);  // update servo position

    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    
    // calculate the distance
    distance_cm = 0.017 * duration_us;
  
    // light up led if distance under limit
    if (distance_cm < 30) {digitalWrite(LED, HIGH);
    }else{digitalWrite(LED, LOW);
    }
    if (distance_cm < 200) {digitalWrite(LED2, HIGH);
    }else{digitalWrite(LED2, LOW);
    }
    if (distance_cm < 399) {digitalWrite(LED3, HIGH); // set to 399 so changes can be seen
    }else{digitalWrite(LED3, LOW);
    }
    
    if (distance_cm != last_distance) {; // if distance is the same dont re-print it to lcd
    lcd.setCursor(0, 0); // start to print at the first row
    lcd.print("Distance: ");
    lcd.print(distance_cm);
    lcd.print(" cm");
    }
    lcd.setCursor(0, 1); // point at second row
    lcd.print("Angle: ");
    lcd.println(pos);
    lcd.print("Degrees");
    //delay(445);

    // print the value to Serial Monitor
    if (distance_cm != last_distance) {;
    Serial.print("distanceA: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    Serial.print("at Angle: ");
    Serial.print(pos);
    Serial.println(" degrees");
    Blink(LED4);
    last_distance = distance_cm;
    int x = ThingSpeak.writeFields(myChannelNumber, myApiKey);  // from example
    if(x == 200){
    Serial.println("Data pushed successfull");
    }else{
    Serial.println("Push error" + String(x));
    }
    Serial.println("-------");
    delay(updateTime);
    }
  }
}

//Using Arduinos Standard Blink Sketch
void Blink(int x){
  digitalWrite(x, HIGH);
  delay(10);
  digitalWrite(x, LOW);
}
