#include <Wire.h>               // Library for I2C communication
#include "rgb_lcd.h"            // RGB LCD SEEED
#include "DHT.h"                // Adafruit dht library
#include "RTClib.h"             // Adafruit RTC library
#include "pitches.h"            // Tone note code
#include <MQ2.h>                // MQ2 gas sensor library.
#include <DallasTemperature.h>  // DS18B20 library. By : Miles Burton, Tim Newsome, and Guil Barros
#include <Adafruit_ADS1015.h>   // Adafruit ADS1115 16 bit ADC library.
#include <TimeLib.h>            // Time library
#include <TimeAlarms.h>         // Library for Set alarm. By : Michael Margolis 2008-2011
#include <CurieIMU.h>           // Library for GYRO
#include <SoftwareSerial.h>     // Serial library for communicate with ESP8266

//SOFTWARE SERIAL
#define rxPin 7
#define txPin 8
SoftwareSerial serial(rxPin, txPin);

// IMU - we use codes from erikyo. Source : https://forum.arduino.cc/index.php?topic=378779.0
float FS_SEL = 131;                                  // IMU gyro values to degrees/sec
unsigned long last_read_time;
float last_x_angle, last_y_angle, last_z_angle;      // These are the filtered angles
float lGXA, lGYA, lGZA;                              // Store the gyro angles to compare drift
// FUNCTIONS
//Math
//Convert radians to degrees
double rtod(double fradians) {
  return (fradians * 180.0 / PI);
}

//Mario main theme melody by: Dipto Pratyaksa
#define melodyPin 4
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

//Mario main them tempo by: Dipto Pratyaksa
int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

/*DEFINE PARAMETER FOR SENSOR AND COMPONENTS*/
Adafruit_ADS1115 ads;   // ADS1115
AlarmId id;            // ALARM FOR SHOWERING THE PLANT
MQ2 mq2(A3);            // MQ02 GAS SENSOR
RTC_DS1307 rtc;         // RTC DS1307
const int pingPin = 6;  // ECHO PIN SRF04
#define motor_air   10
#define motor_soil  11

//D18B20
#define ONE_WIRE_BUS 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

//RGB LCD DEFINITION
rgb_lcd lcd;
const int colorR = 255;
const int colorG = 255;
const int colorB = 0;

//DHT11
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//GLOBAL VARIABEL
char c, ch;
char reply[500];
byte n = 0;
byte hours, hours1, minutes, minutes1, waktu;
int i, j, smoke, smoke_max, waktu1, panjang;

int mois_min, mois_max, SoilTemp_max, WaterLevel_min, WaterLevel_max, AirTemp_max;
int humid_min, humid_max, light_min;
int16_t adc0, adc1, adc2, adc3;
double ads_max = 32767.0;
long duration, cm;
unsigned long microsNow;

float roll, pitch, heading;
float water, soil, light, temp, humid, vibration, water_temp;
String dates, clocks, data;
String result, teks, hasil, PostData, postrequest;
boolean No_IP = false;
String IP = "";

void setup() {
  //SETTING LCD
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);
  lcd.clear();

  //SETTING SERIAL
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  Serial.begin(9600);
  serial.begin(9600);

  //SETTING LED, BUZZER, & MOTOR
  pinMode(2, OUTPUT);        // GREEN LED
  pinMode(3, OUTPUT);        // RED LED
  pinMode(4, OUTPUT);        // BUZZER
  pinMode(A2, OUTPUT);       // GREEN LED
  digitalWrite(A2, HIGH);
  delay(1000);
  pinMode(motor_air, INPUT);  // FAN
  pinMode(motor_soil, INPUT); // MOTOR FOR SHOWERING SOIL & WATER

  //SETTING SENSORS
  dht.begin();  // DHT11
  ads.begin();  // ADS1115
  mq2.begin();  // MQ02

  delay(5000);
  lcd.setCursor(1, 0);
  lcd.print(F("SMART FLOATING"));
  lcd.setCursor(5, 1);
  lcd.print(F("FARM"));
  delay(10000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("----  SFF  -----"));
  lcd.setCursor(0, 1);
  lcd.print("-- MANTIS ID ---");

  //CHECK RED AND GREEN LED
  digitalWrite(2, HIGH);
  delay(3000);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  delay(3000);
  digitalWrite(3, LOW);

  //SERIAL MONITOR CHECK
  Serial.println("Smart Floating Farm - MANTIS ID");

  //GYRO+ACCELEROMETER -------------------
  lcd.clear();
  lcd.print("Calibrate GYRO");
  CurieIMU.begin();
  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieIMU.getXAccelOffset());
  Serial.print("\t");
  Serial.print(CurieIMU.getYAccelOffset());
  Serial.print("\t");
  Serial.print(CurieIMU.getZAccelOffset());
  Serial.print("\t");
  Serial.print(CurieIMU.getXGyroOffset());
  Serial.print("\t");
  Serial.print(CurieIMU.getYGyroOffset());
  Serial.print("\t");
  Serial.println(CurieIMU.getZGyroOffset());
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(1000);
  // The board must be resting in a horizontal position for the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateXAccelOffset(0);
  CurieIMU.autoCalibrateYAccelOffset(0);
  CurieIMU.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true);

  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
  delay(1000);

  //BUZZER PLAY MARIO BROS RINGTONE
  lcd.clear();
  lcd.print("Mario Bros Tone");
  sing();
  delay(1000);

  lcd.clear();
  lcd.print("Check RTC Module");
  //CHECK RTC MODULE
  Serial.println(F("Check RTC module"));
  if (! rtc.begin()) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    Serial.println("Couldn't find RTC");
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print(F("ERROR!")); //Please run the SetTime
    lcd.setCursor(0, 1);
    lcd.print(F("   CONTACT CS"));
    while (1);
  }
  if (! rtc.isrunning()) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    Serial.println("Couldn't find RTC");
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print(F("ERROR!")); //Please run the SetTime
    lcd.setCursor(0, 1);
    lcd.print(F("   CONTACT CS"));
    while (1);
  }

  delay(1000);
  lcd.clear();
  lcd.print("Everything is OK");
  lcd.setCursor(0, 1);
  lcd.print("Warming ESP8266");
  delay(5000);

  // CHECK IF ESP8266 IS CONNECTED AND CAN RECEIVE AT COMMAND
  lcd.clear();
  lcd.print("Finding ESP8266");
  Serial.println("Finding ESP8266");//check via serial monitor
  delay(5000);

  //CHECK IF ESP8266 CAN COMMUNICATE. IF STILL ERROR HIT ESP8266 RESET BUTTON
  while (1) {
    ConnectAT("AT", 100);
    if (result == "OK") {
      break;
    }
  }
  lcd.clear();

  Serial.println(F("Set as station"));
  lcd.print(F("SET AS STATION")); //SET ESP8266 AS STATION MODE
  Connect("AT+CWMODE=1", 100);
  delay(1000);
  lcd.clear();

  Serial.println(F("Disconnecting AP"));
  lcd.print(F("DISCONNECT AP...")); //DISCONNECT FROM AP
  Connect("AT+CWQAP", 100);
  delay(1000);
  lcd.clear();

  lcd.print(F("SET DHCP...")); //SET DHCP
  Connect("AT+CWDHCP=1,1", 100);
  delay(1000);
  lcd.clear();

  Serial.println(F("Connecting WIFI"));
  lcd.print("Connecting Wifi"); //CONNECT TO AP
  //connect_wifi("AT+CWJAP=\"My ASUS holland\",\"dunedin1996\"", 5000);   //provide your WiFi username and password here
  connect_wifi("AT+CWJAP=\"Lenovo X2-AP\",\"16756205\"", 5000);
  delay(1000);
  lcd.clear();

  lcd.print(F("Get IP")); //GET ESP IP ADDRESS
  get_ip();
  Serial.println(IP);
  delay(2000);
  lcd.clear();

  //GREEN LED ACTIVE INDICATE EVERTHING IS OK
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);
}

void loop() {
  unsigned long startTime = 0;
  startTime = millis();
  pinMode(motor_air, INPUT);
  pinMode(motor_soil, INPUT);

  //SET ESP FOR TCP COMMUNICATION
  lcd.print(F("CIPMODE")); //CIPMODE
  Connect("AT+CIPMODE=0", 100);
  delay(1000);
  lcd.clear();
  lcd.print("SingleConnection");
  setnet("AT+CIPMUX=0", 100);
  delay(1000);
  lcd.clear();

  //CONNECT TO SERVER
  lcd.print("CONNECT SERVER");
  InetStart("AT+CIPSTART=\"TCP\",\"www.mantisid.id\",80", 100);
  delay(1000);
  lcd.clear();

  if (result == "error") {
    // ERROR GET SO USE INTERNAL VALUE THRESHOLD
    parameter();
    Serial.println(F("Get default parameter"));
  }
  else {
    // GET PARAMETER FROM SERVER;
    Serial.println(F("Get parameter from server"));
    lcd.print("GET PARAMETER...");
    teks = "GET /api/product/sff_th_r.php?Format=Sys HTTP/1.1\r\nHost: www.mantisid.id\r\n\r\n\r\n";//Connection: close
    panjang = teks.length();
    serial.print("AT+CIPSEND=");
    serial.println(panjang);
    Serial.print("AT+CIPSEND=");
    Serial.println(panjang);
    delay(500);
    serial.print(teks);
    cipsend(5000);
    delay(500);
    lcd.clear();
    lcd.print(F("Close Connection")); //SET ESP8266 AS STATION MODE
    Serial.println("AT+CIPCLOSE");
    serial.println("AT+CIPCLOSE");
    lcd.clear();
    lcd.print("Got Server");
    lcd.setCursor(0, 1);
    lcd.print("Parameter");
    parsingdata();
  }

  //SET TIME FOR SHOWERING THE PLANT
  Alarm.timerRepeat(hours, minutes, 0, siram);    // timer for every 1 seconds
  Alarm.timerRepeat(hours1, minutes1, 0, siram);
  Alarm.delay(0);

  //READ SENSORS
  //USIRR ********************
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  //ping sensor measurement
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  cm = 17 - cm;
  //*********************************

  //DHT11
  humid = dht.readHumidity();
  temp = dht.readTemperature();
  //ANALOG SENSORS
  vibration = analogRead(A0) / 1023.0 * 100.0;
  light = analogRead(A1) / 1023.0 * 100.0;
  smoke = mq2.readSmoke();

  //ADS1115 measurement
  adc0 = ads.readADC_SingleEnded(3);
  adc1 = ads.readADC_SingleEnded(1);
  if (adc0 > 20000) //water level maximum value when all part in water
    adc0 = 20000;
  water = adc0 / 20000.00 * 100.0;
  soil = adc1 / ads_max * 100.0;

  //DS18B20
  sensors.requestTemperatures();
  water_temp = sensors.getTempCByIndex(0);

  //GYRO+ACCELEROMETER ----------------------------------------------------
  unsigned long t_now = millis();
  int ax = CurieIMU.getAccelerationX();
  int ay = CurieIMU.getAccelerationY();
  int az = CurieIMU.getAccelerationZ();
  int gx = CurieIMU.getRotationX();
  int gy = CurieIMU.getRotationY();
  int gz = CurieIMU.getRotationZ();

  // Convert gyro values to degrees/sec
  float gyro_x = gx / FS_SEL;
  float gyro_y = gy / FS_SEL;
  float gyro_z = gz / FS_SEL;

  // Compute the accel angles
  float accelX = rtod(atan(ay / sqrt( pow(ax, 2) + pow(az, 2))));
  float accelY = rtod(-1 * atan(ax / sqrt(pow(ay, 2) + pow(az, 2))));

  // Compute the (filtered) gyro angles
  float dt = (t_now - last_read_time) / 1000.0;
  float gyroX = gyro_x * dt + last_x_angle;
  float gyroY = gyro_y * dt + last_y_angle;
  float gyroZ = gyro_z * dt + last_z_angle;

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x * dt + lGXA;
  float unfiltered_gyro_angle_y = gyro_y * dt + lGYA;
  float unfiltered_gyro_angle_z = gyro_z * dt + lGZA;

  // Apply the complementary filter to figure out the change in angle
  // Alpha depends on the sampling rate...
  float alpha = 0.96;
  pitch = alpha * gyroX + (1.0 - alpha) * accelX;
  roll = alpha * gyroY + (1.0 - alpha) * accelY;
  heading = gyroZ;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, pitch, roll, heading, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  //************************************************************************************************

  //DISPLAY+SEND DATA
  DateTime now = rtc.now();
  dates = String(now.year(), DEC) + '-' + print2digit(now.month()) + '-' + print2digit(now.day());
  clocks = print2digit(now.hour()) + ':' + print2digit(now.minute()) + ':' + print2digit(now.second());

  //COMPILE DATA
  data = "{\"Data\": \"\'" + dates + " " + clocks + "\'";
  data = data + ",'" + String (soil) + "','" + String (water_temp)  + "','" + String (cm) + "','" ;
  data = data + String (water) + "','" + String (temp) + "','" + String (humid) + "','" + String (light);
  data = data + "','" + String (smoke) + "','" + String (vibration) + "','" + String (pitch) + "','" + String (roll) + "','" + String(heading) + "','" + "MANTIS" + "'\"}";

  //SEND TO SERVER
  postrequest = "POST /api/product/sff_dt_c.php HTTP/1.1\r\nHost: www.mantisid.id\r\nAccept: */*\r\nContent-Length: ";
  postrequest = postrequest + data.length() + "\r\n" + "Content-Type: application/x-www-form-urlencoded\r\n" + "\r\n" + data;

  lcd.clear();
  lcd.print("CONNECT SERVER");
  InetStart("AT+CIPSTART=\"TCP\",\"www.mantisid.id\",80", 100);
  delay(1000);
  lcd.clear();
  lcd.print("Send Data:");
  lcd.print(data.length());
  serial.print("AT+CIPSEND=");
  serial.println(postrequest.length());
  delay(100);
  serial.println(postrequest);

  lcd.setCursor(0, 1);
  lcd.print("Sending");

  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("              ");
  lcd.setCursor(0, 1);
  lcd.print("Packet Sent");
  while (serial.available()) {
    String tmpResp = serial.readString();
    Serial.println(tmpResp);
  }

  serial.println("AT+CIPCLOSE");
  lcd.clear();
  lcd.print("AT+CIPCLOSE");
  //SEND TO SERIAL TO CHECK DATA OUTPUT
  Serial.print(data);
  Serial.print("|Time ");
  delay(1000);

  //DISPLAY ON LCD
  lcdprint();
  //SET OUTPUT BASED ON THRESHOLD
  output();
  delay(1000);
  lcd.clear();
  lcd.print(F("- SFF MANTISID -"));
  lcd.setCursor(0, 1);
  lcd.print(F("Wait 4 Next Data"));
  i = 0;
  waktu1 = millis() - startTime;
  Serial.print(waktu1);
  waktu1 = waktu * 60 * 1000 - waktu1; waktu1 = 60000;
  Serial.print(" ");
  Serial.println(waktu1);

  //delay for next data, while delay keep record gyro to maintan the data
  while (i < waktu1) {
    //GYRO+ACCELEROMETER ----------------------------------------------------
    unsigned long t_now = millis();
    int ax = CurieIMU.getAccelerationX();
    int ay = CurieIMU.getAccelerationY();
    int az = CurieIMU.getAccelerationZ();
    int gx = CurieIMU.getRotationX();
    int gy = CurieIMU.getRotationY();
    int gz = CurieIMU.getRotationZ();

    // Convert gyro values to degrees/sec
    float gyro_x = gx / FS_SEL;
    float gyro_y = gy / FS_SEL;
    float gyro_z = gz / FS_SEL;

    // Compute the accel angles
    float accelX = rtod(atan(ay / sqrt( pow(ax, 2) + pow(az, 2))));
    float accelY = rtod(-1 * atan(ax / sqrt(pow(ay, 2) + pow(az, 2))));

    // Compute the (filtered) gyro angles
    float dt = (t_now - last_read_time) / 1000.0;
    float gyroX = gyro_x * dt + last_x_angle;
    float gyroY = gyro_y * dt + last_y_angle;
    float gyroZ = gyro_z * dt + last_z_angle;

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + lGXA;
    float unfiltered_gyro_angle_y = gyro_y * dt + lGYA;
    float unfiltered_gyro_angle_z = gyro_z * dt + lGZA;

    // Apply the complementary filter to figure out the change in angle
    // Alpha depends on the sampling rate...
    float alpha = 0.96;
    pitch = alpha * gyroX + (1.0 - alpha) * accelX;
    roll = alpha * gyroY + (1.0 - alpha) * accelY;
    heading = gyroZ;  //Accelerometer doesn't give z-angle

    // Update the saved data with the latest values
    set_last_read_angle_data(t_now, pitch, roll, heading, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    delay(1);
    i++;
  }
}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x; last_y_angle = y; last_z_angle = z;
  lGXA = x_gyro; lGYA = y_gyro; lGZA = z_gyro;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

String print2digit(byte num) {
  String data;
  if (num >= 0 && num < 10)
  {
    data = '0' + String(num);
  }
  else
  {
    data = String(num);
  }
  return data;
}

void siram() { //showering plant for 1 minute
  for (byte i = 0; 1 < 6; i++) {
    pinMode(motor_soil, OUTPUT);
    digitalWrite(motor_soil, HIGH);
    delay(3 * 1000);
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    delay(2 * 1000);
  }
}

void parameter() {
  hours = 7;      //hour for showering plant in the morning
  hours1 = 16;      //hour for showering plant in the evening
  minutes = 0;      //minute for showering plant in the morning
  minutes1 = 0;     //minute for showering plant in the evening
  waktu = 5;      //delay for next data
  mois_min = 30;    //minimum threshold for soil moisture
  mois_max = 75;    //maximum threshold for soil moisture
  SoilTemp_max = 35;  //maximum threshold for soil temperature
  WaterLevel_min = 20;  //minimum threshold for water level
  WaterLevel_max = 75;  //maximum threshold for water level
  AirTemp_max = 33;   //maximum threshold for air temperature
  humid_min = 40;   //minimum threshold for humid
  humid_max = 75;   //maximum threshold for humid
  light_min = 15;   //minimum threshold for light intensity
  smoke_max = 2000;   //maximum threshold for smoke
  lcd.print("Default");
  lcd.setCursor(0, 1);
  lcd.print("Parameter");
}

void sing() { //play mario bros tune
  // iterate over the notes of the melody:
  int size = sizeof(melody) / sizeof(int);
  for (int thisNote = 0; thisNote < size; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / tempo[thisNote];

    buzz(melodyPin, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // stop the tone playing:
    buzz(melodyPin, 0, noteDuration);
  }
}

void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}

void lcdprint() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(dates);
  lcd.setCursor(0, 1);
  lcd.print(clocks);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("soil="));
  lcd.print(soil);
  lcd.setCursor(0, 1);
  lcd.print(F("water temp="));
  lcd.print(water_temp, 2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("height="));
  lcd.print(cm);
  lcd.setCursor(0, 1);
  lcd.print(F("water lvl="));
  lcd.print(water, 2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("temp="));
  lcd.print(temp, 2);
  lcd.setCursor(0, 1);
  lcd.print(F("humid="));
  lcd.print(humid, 2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("light="));
  lcd.print(light, 2);
  lcd.setCursor(0, 1);
  lcd.print(F("smoke="));
  lcd.print(smoke);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("vibrate="));
  lcd.print(vibration, 2);
  lcd.setCursor(0, 1);
  lcd.print(F("pitch="));
  lcd.print(pitch, 2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("roll="));
  lcd.print(roll, 2);
  lcd.setCursor(0, 1);
  lcd.print(F("heading="));
  lcd.print(heading, 2);
}

void output() {
  //SOIL
  if (soil < mois_min || water_temp > SoilTemp_max || water < WaterLevel_min) {// nyalakan motor
    pinMode(motor_soil, OUTPUT);
    digitalWrite(motor_soil, HIGH);
    digitalWrite(2, HIGH);
    tone(4, NOTE_A4, 1000);
    delay(1000);
    noTone(4);
  }
  //AIR
  if (temp >= AirTemp_max || humid < humid_min || smoke >= smoke_max) {
    pinMode(motor_air, OUTPUT);
    digitalWrite(motor_air, HIGH);
    digitalWrite(2, HIGH);
    tone(4, NOTE_C6, 1000);
    delay(1000);
    noTone(4);
  }
  //LIGHT
  if (light < light_min) {
    digitalWrite(A2, LOW);
  }
  delay(10000);
  digitalWrite(motor_air, LOW);
  pinMode(motor_air, INPUT);
  //TURN OFF MOTOR & LED
  if (soil == mois_max || temp < (SoilTemp_max - 2) || water > WaterLevel_max) {
    digitalWrite(motor_soil, LOW);
    pinMode(motor_soil, INPUT);
  }

  if (temp < (AirTemp_max - 2) && humid > humid_max && smoke >= smoke_max) {
    digitalWrite(motor_air, LOW);
    pinMode(motor_air, INPUT);
  }
  if (light > light_min) {
    digitalWrite(A2, HIGH);
  }
}

void ConnectAT(String cmd, int t) {
  int i = 0;
  while (1) {
    serial.println(cmd);
    while (serial.available()) {
      if (serial.find("OK"))
        i = 8;
    }
    delay(t);
    if (i > 5) {
      break;
    }
    i++;
  }
  if (i == 8) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("OK");
    result = "OK";
    delay(1000);
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("ESP8266 ERROR");
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
    delay(500);
    lcd.setCursor(0, 1);
    lcd.print("                ");
    delay(100);
  }
}

void Connect(String cmd, int t) {
  int i = 0;
  while (1)
  {
    serial.println(cmd);
    while (serial.available())
    {
      if (serial.find("OK"))
        i = 8;
    }
    delay(t);
    if (i > 5)
    {

      break;
    }
    i++;
  }
  if (i == 8) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("OK");
    result = "OK";
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("SET ERROR");
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
  }
}

void connect_wifi(String cmd, int t) {
  int i = 0;
  while (1)
  {
    serial.println(cmd);
    while (serial.available())
    {
      if (serial.find("WIFI CONNECTED"))
        i = 8;
      else {
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print("WIFI not connect");
      }
    }
    delay(t);
    if (i > 5)
      break;

  }
  if (i == 8) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("WIFI Connected..");
    delay(1000);
  }
}

void get_ip() {
  IP = "";
  c = 0;

  while (1)
  {
    serial.println("AT+CIFSR");
    while (serial.available() > 0)
    {
      if (serial.find("STAIP,"))
      {
        delay(1000);
        while (serial.available() > 0)
        {
          c = serial.read();
          if (c == '+')
            break;
          IP += ch;
        }
      }
      if (c == '+')
        break;
    }
    if (c == '+')
      break;
    delay(100);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(IP);
}

void setnet(String cmd, int t) {
  int i = 0;
  while (1)
  {
    serial.println(cmd);
    while (serial.available())
    {
      if (serial.find("OK"))
        i = 8;
    }
    delay(t);
    if (i > 5)
    {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print("WIFI ERROR");
      digitalWrite(3, LOW);
      digitalWrite(2, HIGH);
      break;
    }
    i++;
  }
  if (i == 8) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("OK");
  }
}

void InetStart(String cmd, int t) {
  int i = 0;
  while (1)
  {
    serial.println(cmd);
    delay(100);
    while (serial.available())
    {
      if (serial.find("CONNECT"))
        i = 8;
    }
    delay(t);
    if (i > 5)
      break;
    i++;
  }
  if (i == 8) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("OK");
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("Error");
    result = "error";
  }
}

void cipsend(int wait) {
  IP = "";
  ch = 0;
  int tempPos = 0;
  long int waktu = millis();
  while ((waktu + wait) > millis()) {
    while (serial.available())
    {
      ch = serial.read();

      if (tempPos < 500) {
        reply[tempPos] = ch;
        tempPos++;
      }
      if (ch == ']')
        break;
    }
    reply[tempPos] = 0;
  }

  hasil = String(reply);
}

void parsingdata() {
  int Bracket = hasil.indexOf('#');
  int bracket = hasil.lastIndexOf('%');

  teks = hasil.substring(Bracket + 1, bracket);
  Serial.println();
  Serial.println(teks);
  delay(100);
  hasil = teks.substring(0, 2);
  hours = hasil.toInt();
  hasil = teks.substring(2, 4);
  minutes = hasil.toInt();
  hasil = teks.substring(5, 7);
  hours1 = hasil.toInt();
  hasil = teks.substring(7, 9);
  minutes1 = hasil.toInt();
  hasil = teks.substring(10, 11);
  waktu = hasil.toInt();
  hasil = teks.substring(12, 14);
  mois_min = hasil.toInt();
  hasil = teks.substring(15, 17);
  mois_max = hasil.toInt();
  hasil = teks.substring(18, 20);
  SoilTemp_max = hasil.toInt();
  hasil = teks.substring(21, 23);
  WaterLevel_min = hasil.toInt();
  hasil = teks.substring(24, 26);
  WaterLevel_max = hasil.toInt();
  hasil = teks.substring(27, 29);
  AirTemp_max = hasil.toInt();
  hasil = teks.substring(30, 32);
  humid_min = hasil.toInt();
  hasil = teks.substring(33, 35);
  humid_max = hasil.toInt();
  hasil = teks.substring(36, 38);
  light_min = hasil.toInt();
  hasil = teks.substring(39, 43);
  smoke_max = hasil.toInt();

  //CHECK FROM SERIAL MONITOR: IS THRESHOLD VALUE CORRECT
  Serial.print(hours);
  Serial.print(" ");
  Serial.println(minutes);
  Serial.print(hours1);
  Serial.print(" ");
  Serial.println(minutes1);
  Serial.println(waktu);
  Serial.print(mois_min);
  Serial.print(" ");
  Serial.println(mois_max);
  Serial.println(SoilTemp_max);
  Serial.print(WaterLevel_min);
  Serial.print(" ");
  Serial.println(WaterLevel_max);
  Serial.println(AirTemp_max);
  Serial.print(humid_min);
  Serial.print(" ");
  Serial.println(humid_max);
  Serial.println(light_min);
  Serial.println(smoke_max);
}



