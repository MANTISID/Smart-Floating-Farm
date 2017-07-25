# Smart-Floating-Farm
Future Farming Platform in Coastal and Remote Island. Cultivate the future, start from the sea. We submit this project to  Intel Hackathon 2017

## Inspiration
**We want to help people**

We see that local people in Coastal area and Remote Islands having difficulty in planting horticulture because they face the problem of land limitations. We have an idea about making floating platform so people can grow their vegetables at coastal area.
We inspired by [**GROASIS**](https://www.groasis.com/en), [**Seawater Greenhouse**](https://seawatergreenhouse.com/), and [**Hamster Ball-Shaped Solarball**](http://inhabitat.com/hamster-ball-shaped-solarball-uses-the-sun-to-purify-water/). They can grow plants at desert area and have a pure water using desalination process.
Hopefully this platform can help local people to reduce their dependence on supply from mainland, help them make their own food independence and also help to make more money from selling the harvest vegetables. We also can manage these process with supllying seed, provide consultation to user, and preparing the market to sell the vegetables.

## What it does
__**SMART FLOATING FARM (SFF) PLATFORM BUILD TO RECORD, MONITOR, AND MAKE DECISION BASED ON THE RECORD.**__

__**Initialize sensors and output**__

This platform will initialize serial, lcd, RTC (real time clock), sensors, and motor. If RTC cannot be detected, LCD will display error and user must fix it. After RTC is ok, Arduino101 will check communication with ESP8266. Arduino101 send "AT" and wait "OK" from ESP8266.
 
__**Get threshold from Server**__

After receiving "OK", Arduino will command ESP8266 as station mode and get parameter threshold from Server.  These threshold value used to ensure plants are in the range that has been determined by the user. SFF send GET method to server ([This link](http://mantisid.id/api/product/sff_th_r.php?Format=Sys) used to receive sensors threshold). Parameter threshold will be saved as variable.

__**Sensor Measurement**__

This platform record these parameters : soil moisture & temperature, greenhouse temperature & humidity, greenhouse smoke detection, vibration, plant height, time, and light intensity.
 
__**Sending Data to Server**__

Data from sensor compiled as String and send it to Web by ESP8266 using POST method ([Go to this link](http://mantisid.id/api/product/sff_dt_c.php) to send data to WebServer; [Visit this link](http://mantisid.id/api/product/sff_dt_r.php) to see data that received by WebServer from Platform).

__**Output Action**__

Arduino101 check data measurement with threshold received from Server. Based on these threshold, platform will give command automatically to activate LED indicator, water pump, exhaust fan and LED lamps based on sensors reading. After 10 seconds, platform will delay few minutes before the next measurement. While waiting, Arduino101 measure gyro value. 

## How we built it
We use Arduino IDE to built the firmware. We use libraries from internal Arduino IDE and from internet. For external library, we used  [SEEED RGB LCD ](https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight), [Adafruit DHT11 ](https://github.com/adafruit/DHT-sensor-library), [Adafruit RTC ](https://github.com/adafruit/RTClib), [Adafruit ADS1115 ](https://github.com/adafruit/Adafruit_ADS1X15), [MQ02 Gas Sensor](https://github.com/labay11/MQ-2-sensor-library), [Dallas DS18B20 ](https://github.com/milesburton/Arduino-Temperature-Control-Library), [Rotation from Gyro & Accelerometer](https://forum.arduino.cc/index.php?topic=378779.0), [Time Alarm ](https://github.com/michaelmargolis/arduino_time/tree/master/Time), and [Time Lib ](https://github.com/PaulStoffregen/Time). For internal library, we used Wire, pitches.h, CurieIMU and Software Serial. We used Mario Bros Tune developed by [Dipto Pratyaksa](http://www.linuxcircle.com/2013/03/31/playing-mario-bros-tune-with-arduino-and-piezo-buzzer/) to play sound at the beginning. IMU rotation angle we got from [Erikyo](https://forum.arduino.cc/index.php?topic=378779.0). SFF Arduino sketch can be found at [MANTIS GITHUB](https://github.com/MANTISID/Smart-Floating-Farm).

## Sensor and output configuration
**ARDUINO101   SENSOR/OUTPUT**
- D0    <---->    Tx UART
- D1    <---->    Rx UART
- D2    <---->    GREEN LED INDICATOR
- D3    <---->    RED LED INDICATOR
- D4    <---->    BUZZER
- D5    <---->    DHT11
- D6    <---->    SRF04 ECHO PIN
- D7    <---->    Tx ESP8266
- D8    <---->    Rx ESP8266
- D9    <---->    DS18B20
- D10   <---->    RELAY FOR EXHAUST FAN & SPRAYER
- D11   <---->    RELAY FOR WATER PUMP
- A0    <---->    VIBRATION SENSOR
- A1    <---->    LIGHT SENSOR
- A2    <---->    GREEN LED FOR LIGHTING
- A3    <---->    MQ02 GAS SENSOR

- A4    <---->    SDA
- A5    <---->    SCL

**I2C COMMUNICATION**
- RGB LCD SHIELD
- ADS1115 ADC 16 BIT
    * ADC1    <---->    SOIL MOISTURE SENSOR
    * ADC3    <---->    WATER LEVEL SENSOR
    
**INTERNAL USE:**
- INERTIAL MEASUREMENT UNIT (GYRO & ACCELEROMETER)
