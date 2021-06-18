// Tutorial 13e. Decibel monitor with ESP32 and Adafruit IO

// Main parts: Adafruit Metro Mini, Adafruit AirLift FeatherWing ESP32,
// DFrobot Gravity: Analog Sound Level Meter, NeoPixel Ring 24 x 5050
// RGB LED, two 10k trim potentiometers, free AIO subscription

// Libraries required to interface with the transceiver via SPI, to
// manage WLAN and MQTT communication and to drive RGB LEDs; use the
// latest versions
#include <SPI.h> // Serial Peripheral Interface, a communication protocol
#include <WiFiNINA.h> // Adafruit's WiFiNINA fork, use version 1.4.0
#include "Adafruit_MQTT.h" // Message Queuing Telemetry Transport, a communication protocol
#include "Adafruit_MQTT_Client.h"
#include "FastLED.h" // RGB LED control, use Systembolaget fork at GitHub

// Variables that remain constant
// WiFi
#define SPIWIFI SPI // SPI port
#define SPIWIFI_SS 4 // AirLift ESP32 chip select pin
#define ESP32_RESET 3 // AirLift ESP32 reset pin
#define SPIWIFI_ACK 2 // AirLift ESP32 ready pin
#define ESP32_GPIO0 -1 // AirLift ESP32 pin not used
#define WLAN_SSID "#" // WLAN router SSID
#define WLAN_PASS "#" // WLAN router key
//#define WLAN_SSID "#" // Smartphone hotspot SSID
//#define WLAN_PASS "#" // Smartphone hotspot key
// AIO
#define AIO_SERVER "io.adafruit.com" // MQTT broker/server host
#define AIO_SERVERPORT 8883 // Secure port, 1883 insecure port
#define AIO_USERNAME "#" // AIO user name
#define AIO_KEY "#" // AIO key

const int intervalSEN0232 = 2000; // MQTT broker publish interval

const byte pinData = 6; // Digital output pin to LED ring
const byte pinSEN0232 = A0; // Analog input pin from decibel meter
const byte pinPotentiometerMin = A1; // Analog input pin from trim potentiometer 1
const byte pinPotentiometerMax = A2; // Analog input pin from trim potentiometer 2

const byte ledCount = 24; // Number of the Neopixel ring's LEDs
const byte ledBrightness = 192; // Maximum brightness 0 - 255
struct CRGB ledRing[ledCount]; // Array storing the LED's data
struct CRGB ledGradient[ledCount]; // Array storing the LED's colours

const int intervalPeakHold = 900; // Peak hold interval in milliseconds
const int intervalPeakDecay = 50; // Peak decay interval in milliseconds

const float EMA_a = 0.8; // EMA (exponential moving average) alpha factor; lower = smoother = but more lag

// Instances objects from the FastLED library to set the LED ring's colours
CHSV gradientHueStart = CHSV(96, 255, 96); // Darker at the bottom
CHSV grandientHueEnd = CHSV(0, 255, 192);

// Variables that can change
unsigned long timeSEN0232 = 0; // Timestamp that updates each loop() iteration

long dBValue = 0; // Stores the decibel value
int dBValueEMA = 0; // Stores the EMA value of the dBValue
byte dBMin = 0; // Lowest dB value, adjustable with potentiometer 1
byte dBMax = 0; // Highest dB value, adjustable with potentiometer 2

int newPeak = 0; // Peak value (= number of LEDs from 0 to peak)
int previousPeak = 0;
unsigned long timePeak = 0; // Timestamp that updates with each new peak
byte brightnessPeak = 0; // Stores the peak LED's brightness
bool decay = false; // Toggles if the peak LED can decay or not

bool stateLED = LOW; // Tracks if the AirLift FeatherWing LED is on/off
unsigned long timeLED = 0; // Timestamp that updates each loop() iteration
unsigned long intervalLED = 0; // Tracks the AirLift FeatherWing LED flash interval
bool triggerFlash = false; // Tracks if flashing was triggered

uint8_t status = WL_IDLE_STATUS;

// Instances an object from the WiFiNINA library to connect and
// transfer data with SSL/TLS support
WiFiSSLClient client;

// Instances a client object from the MQTT_Client library with a
// WLAN client, MQTT server, port and credentials
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Instances a publishing object from the MQTT_Client library
Adafruit_MQTT_Publish SEN0232feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dB");

void setup()
{
  // Override default pins with the AirLift's breakout board pins
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESET, ESP32_GPIO0, &SPIWIFI);

  // Indicate that there is no WLAN connection
  WiFi.setLEDs(0, 255, 0); // Red

  // Initialise the FastLED library with the type of programmable RGB LED
  // used, the digital output pin the LED ring is wired to, the array that
  // holds each LED's data, and the number of LEDs in the ring
  FastLED.addLeds<NEOPIXEL, pinData>(ledRing, ledCount);

  FastLED.setBrightness(ledBrightness);

  // Fill the LED's colour array with a colour gradient
  fill_gradient(ledGradient, 0, gradientHueStart, 23, grandientHueEnd, SHORTEST_HUES);
}

void loop()
{
  // A call to this function connects to the WLAN router
  connectToWLAN();

  // A call to this function connects to the MQTT broker
  connectToMQTT();

  // A call to this function fetches a reading from the sensor
  readSEN0232();

  // A call to this function fetches readings from the potentiometers
  readPotentiometers();

  // When it is time to publish a sensor value
  if (millis() - timeSEN0232 >= intervalSEN0232)
  {
    timeSEN0232 = millis(); // Update the timestamp for the next loop() iteration

    SEN0232feed.publish((long)dBValue); // Then publish the data to the AIO feed

    triggerFlash = true; // And indicate that data will be published
  }

  // If an event triggered flashing
  if (triggerFlash == true)
  {
    // Then flash the LED with different on and off intervals n times
    flash(125, 75, 3, 255, 255, 255); // White
  }

  // The three following blocks of code contain the peak hold and
  // peak decay logic, based on a hold and a decay interval

  // If the new peak value is higher than the previous peak value
  if (newPeak >= previousPeak)
  {
    previousPeak = newPeak; // Set the previous peak value to the new peak value

    timePeak = millis(); // And take a timestamp of when that happened

    brightnessPeak = ledBrightness; // Then set the peak LED's brightness to that of all LEDs

    decay = false; // And set the decay flag to false to hold the peak LED in the current position on the ring
  }

  // If the peak LED is not decaying and the peak hold interval (here 900 ms) has expired
  else if (!decay && (millis() - timePeak >= intervalPeakHold))
  {
    timePeak += intervalPeakHold; // Add the expired interval to the timestamp

    decay = true; // And set the peak LED to decay, so its position on the ring and its brightness can change
  }

  // If the peak LED is set to decay and a decay interval (here 50 ms) has passed
  else if (decay && (millis() - timePeak > intervalPeakDecay))
  {
    // And only while the peak LED has not yet dropped to position zero the ring
    if (previousPeak > 0)
    {
      previousPeak --; // Lower the peak LED's position on the ring by one

      // If the peak LED's brightness has dropped to or below zero
      if (brightnessPeak <= 0)
      {
        brightnessPeak = 0; // Then keep it off
      }

      // If the peak LED's brightness has not yet dropped to zero
      else
      {
        brightnessPeak -= 16; // Then reduce the peak LED's brightness
      }

      timePeak += intervalPeakDecay; // And add the expired decay interval to the timestamp
    }
  }

  // Clear the LED array that holds all LED's colour and brightnesss data
  FastLED.clear();

  for ( byte i = 0; i <= newPeak; i++)
    // And depending on the newe peak value (= number of LEDs to light)
  {
    // Copy the corresponding colour from the gradient array to the LED array
    ledRing[i] = ledGradient[i];
  }

  // And also set the peak hold LED's colour and brightness
  ledRing[previousPeak] = CHSV(0, 255, brightnessPeak); // Red

  // Then display all LED's data on the ring
  FastLED.show();
}

void connectToWLAN()
{
  // Return to loop() if already connected to the WLAN router
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  // Connect to the Wifi router
  do
  {
    // Indicate that there is no WLAN connection
    WiFi.setLEDs(0, 255, 0); // Red

    // Start the connection
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);

    // Wait until connected
    delay(100);

    // Repeat as long as WiFi status returns "not connected"
  } while (status != WL_CONNECTED);

  // Indicate that the WiFi connection is active
  WiFi.setLEDs(176, 255, 0); // Yellow
}

void connectToMQTT()
{
  // Stores a printable string version of the error code returned by
  // connect()
  int8_t MQTTerrorString;

  // Number of connection attempts before a hard reset is necessary
  uint8_t retries = 3;

  // Return to loop() if already connected to the MQTT broker
  if (mqtt.connected())
  {
    return;
  }

  // In case the error code is not 0 = successful connection, then
  while ((MQTTerrorString = mqtt.connect()) != 0)
  {
    // Send a MQTT disconnect packet and break the connection
    mqtt.disconnect();

    // And wait for 3 seconds, retry to connect
    delay(3000);

    // Then decrement the retry-counter
    retries--;

    // If no MQTT broker connection can be (re-)established
    if (retries == 0)
    {
      // Then indicate that the connection permanently failed
      WiFi.setLEDs(0, 255, 0); // Red

      // End and wait for user to press the reset button
      while (1)
        ;
    }
  }
  // Indicate that the MQTT connection is active
  WiFi.setLEDs(255, 0, 0); // Green
}

void readSEN0232()
{
  // Stores the sensor's 0,6 - 2,6V voltage output (30 - 130 dB)
  float voltageValue;

  // Calculation of the decibel value from voltage read; 5,0V reference,
  // because of the Adafruit Metro Mini's maximum voltage output
  voltageValue = analogRead(pinSEN0232) / 1024.0 * 5.0;
  dBValue = voltageValue * 50.0;

  // Calculate an exponential moving average (EMA) for signal smoothing
  dBValueEMA = int ((EMA_a * dBValue) + ((1 - EMA_a) * dBValueEMA)) + 0.5;

  // Scale the smoothed dB reading to the minimum and maximum values,
  // to a position of 0 - 23 on the ring, and clip out-of-range values
  newPeak = constrain(map(dBValueEMA, dBMin, dBMax, 0, 23), 0, 23);
}

void readPotentiometers()
{
  // Read the voltages from the potentiometer pins and scale them to
  // a minimum and maximum value. If the location is generally too
  // loud or too quiet, this will ensure that still all 24 LEDs will
  // be used for display. The published dBValue is not affected
  dBMin = constrain(map(analogRead(pinPotentiometerMin), 0, 1023, 35, 60), 35, 60);
  dBMax = constrain(map(analogRead(pinPotentiometerMax), 0, 1023, 60, 130), 60, 130);
}

void flash(int timeon, int timeoff, byte flashes, byte g, byte r, byte b)
{
  // A variable to count how often the LED flashed (on/off). The static
  // keyword preserves a variable's value between function calls, unlike
  // a local variable declared and destroyed at each new function call
  static byte counter = 0;

  // Check if it is time to flash the LED
  if (millis() - timeLED > intervalLED)
  {
    // Create a new timestamp for the next loop() execution
    timeLED = millis();

    // First check, if the LED was off (= LOW); if it was
    if (stateLED == LOW)
    {
      // Use the on-time set in the function call
      intervalLED = timeon;

      // Then switch the LED on with the specified colours
      WiFi.setLEDs(g, r, b);

      // And remember that it is now on
      stateLED = HIGH;
    }

    // Otherwise, if the LED was on (= HIGH)
    else
    {
      // Use the off-time set in the function call
      intervalLED = timeoff;

      // Then switch the LED off
      WiFi.setLEDs(0, 0, 0);

      // And remember that it is now off
      stateLED = LOW;

      // Finally increment the counter variable at each on/off cycle
      counter++;
    }
  }

  // Check if the number of on/off cycles matches the number of flashes
  // set in the function call and if it does
  if (counter >= flashes)
  {
    // Reset the flash cycle counter to zero
    counter = 0;

    // And stop the switch triggering flashes, if the user continues
    // holding the momentary switch button down. This ensures there
    // is only a "single shot"/"one shot" operation
    triggerFlash = false;

    // Finally set LED back to WLAN and MQTT connected colour
    WiFi.setLEDs(255, 0, 0);
  }
}
