# Physical computing wireless tutorial - Decibel monitor with ESP32 and Adafruit IO

Easy Arduino TX. Monitor noisy location dB with peak hold and decay, display on a NeoPixel RGB LED ring with FastLED, and transmit to Adafruit IO

### Result

You can click the image to play the associated YouTube video

[![Alt text](Assets/13e%20result.jpg)](https://www.youtube.com/watch?v=4H6Es9fJvsU)

### Schematic

![](Assets/13e%20schematic.png)

### Peak hold and decay principle

![](Assets/Peak%20hold%20and%20decay.png)

### BOM

<pre>
€  13,00 Adafruit Metro Mini 328 5V 16MHz microcontroller
€  14,00 Adafruit AirLift FeatherWing - ESP32 WiFi
€  40,00 DFrobot Gravity: Analog Sound Level Meter
€  16,00 Adafruit NeoPixel Ring 24 x 5050 RGB LED
€   2,00 2 Breadboard trim potentiometers 10kΩ
€   8,00 2 Half-size transparent breadboards
€   1,00 Jumper cables
€   1,00 2,1mm DC barrel-jack
€   1,00 100 µF 10V el. cap
€  13,00 MEANWELL GS12E05-P1I PSU
€ 109,00
</pre>  

### Useful links

μc https://www.adafruit.com/product/2590  
WiFi module https://www.adafruit.com/product/4264  
Adafruit WiFiNINA library https://github.com/adafruit/WiFiNINA  
Adafruit MQTT library https://github.com/adafruit/Adafruit_MQTT_Library  
Sensor https://wiki.dfrobot.com/Gravity__Analog_Sound_Level_Meter_SKU_SEN0232  
LED ring https://learn.adafruit.com/adafruit-neopixel-uberguide/downloads  
LED library (WiFi compatible fork) https://github.com/systembolaget/FastLED  
