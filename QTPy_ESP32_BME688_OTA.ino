/**
 * @brief This sketch will use a BME688 sensor to show temperature, pressure, humidity, and gas readings.
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 *
 * My topic formats are:
 *    <location>/<device>/<device reading>
 *    <location>/<device>/<sensor type>/<sensor reading>
 *
 * @copyright   Copyright © 2022 Adam Howell
 * @license     The MIT License (MIT)
 */
#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include <ESP8266WiFi.h> // ESP8266 Wifi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h> // OTA - Multicast DNS for the ESP8266.
#elif ESP32
// These headers are installed when the ESP32 is installed in board manager.
#include <WiFi.h>		// ESP32 Wifi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h> // OTA - Multicast DNS for the ESP32.
#else
#include <WiFi.h> // Arduino Wifi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#endif
#include <WiFiUdp.h>					// OTA - Library to send and receive UDP packets.
#include <ArduinoOTA.h>				// OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>			// A JSON manipulation library.  Author: Benoît Blanchon  https://github.com/bblanchon/ArduinoJson  https://arduinojson.org/
#include <Adafruit_NeoPixel.h>	// The Adafruit NeoPixel library to drive the RGB LED on the QT Py.	https://github.com/adafruit/Adafruit_NeoPixel
#include <Adafruit_BME680.h>		// Adafruit BME680 (and BME688) library.  https://github.com/adafruit/Adafruit_BME680
#include "privateInfo.h"			// Contains passwords, settings, and API keys.  Not uploaded to GitHub.


// NeoPixel related values.
#define NUMPIXELS 1
#define RED       0xFF0000
#define ORANGE    0xFFA500
#define YELLOW    0xFFFF00
#define GREEN     0x00FF00
#define BLUE      0x0000FF
#define INDIGO    0x4B0082
#define VIOLET    0xEE82EE
#define PURPLE    0x800080
#define BLACK     0x000000
#define GRAY      0x808080
#define WHITE     0xFFFFFF


/*
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Typically declared in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Typically declared in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Typically declared in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Typically declared in "privateInfo.h".

const char * hostName = "ESP32_Soil_OTA";												// The hostname used for OTA access.
const char * notes = "Adafruit QT Py ESP32-S2 with BME688 and OTA";			// Notes sent in the bulk publish.
const char * commandTopic = "livingRoom/QTPy/command";							// The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char * sketchTopic = "livingRoom/QTPy/sketch";								// The topic used to publish the sketch name.
const char * macTopic = "livingRoom/QTPy/mac";										// The topic used to publish the MAC address.
const char * ipTopic = "livingRoom/QTPy/ip";											// The topic used to publish the IP address.
const char * rssiTopic = "livingRoom/QTPy/rssi";									// The topic used to publish the WiFi Received Signal Strength Indicator.
const char * publishCountTopic = "office/QTPy/publishCount";					// The topic used to publish the loop count.
const char * notesTopic = "livingRoom/QTPy/notes";									// The topic used to publish notes relevant to this project.
const char * tempCTopic = "livingRoom/QTPy/bme688/tempC";						// The topic used to publish the temperature in Celsius.
const char * tempFTopic = "livingRoom/QTPy/bme688/tempF";						// The topic used to publish the temperature in Fahrenheit.
const char * pressureHPaTopic = "livingRoom/QTPy/bme688/pressureHPa";		// The topic used to publish the barometric pressure in hectopascals/millibars.
const char * humidityTopic = "livingRoom/QTPy/bme688/humidity";				// The topic used to publish the humidity.
const char * gasResistanceTopic = "livingRoom/QTPy/bme688/gasResistance";	// The topic used to publish the gas resistance.
const char * altitudeMTopic = "livingRoom/QTPy/bme688/altitudeM";				// The topic used to publish the altitude (derived from barometric pressure).
const char * mqttStatsTopic = "espStats";												// The topic this device will publish to upon connection to the broker.
const char * mqttTopic = "espWeather";													// The topic used to publish a single JSON message containing all data.
const int JSON_DOC_SIZE = 512;															// The ArduinoJson document size.
unsigned long publishInterval = 60000;													// The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long lastPublishTime = 0;														// Stores the time of the last MQTT publish.
unsigned long sensorPollInterval = 10000;												// The delay between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long lastPollTime = 0;															// Stores the time of the last sensor poll.
unsigned long mqttReconnectInterval = 5000;											// The time between MQTT connection attempts.
unsigned int networkIndex = 2112;														// An unsigned integer to hold the correct index for the network arrays: wifiSsidArray[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
unsigned long wifiConnectionTimeout = 10000;											// The maximum amount of time in milliseconds to wait for a WiFi connection before trying a different SSID.
unsigned long publishCount = 0;															// A count of how many publishes have taken place.
char ipAddress[16];																			// The IPv4 address of the WiFi interface.
char macAddress[18];																			// The MAC address of the WiFi interface.
long rssi;																						// A global to hold the Received Signal Strength Indicator.
float tempC;																					// The sensor temperature in Celsius.
float tempF;																					// The sensor temperature in Fahrenheit.
float pressureHPa;																			// The sensor barometric pressure in hectopascals (millibars).
float humidity;																				// The sensor relative humidity as a percetage.
float gasResistance;																			// The sensor VOC level represented as electrical resistence of a MOX sensor in Ohms.
float altitudeM;																				// The sensor estimated altitude in meters.
float SEALEVELPRESSURE_HPA = 1025.0;													// This is the barometric sea-level pressure for the sensor's location.


// Class objects.
WiFiClient wifiClient;																			// Network client.  Used only by the MQTT client.
PubSubClient mqttClient( wifiClient );														// MQTT client to manage communication to the broker.
Adafruit_NeoPixel pixels( NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800 );		// An object to manage the onboard RGB LED.
Adafruit_BME680 bme;																				// The Bosche BME688 atmospheric sensor.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	delay( 1000 ); // A pause to give me time to open the serial monitor.
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Serial.println( "\nSetup is initializing hardware and configuring software." );

#if defined( NEOPIXEL_POWER )
	// If this board has a power control pin, we must set it to output and high in order to enable the NeoPixels.
	// We put this in an #ifdef so it can be reused for other boards without compilation errors.
	pinMode( NEOPIXEL_POWER, OUTPUT );
	digitalWrite( NEOPIXEL_POWER, HIGH );
#endif
	// Initialize the NeoPixel.
	pixels.begin();
	pixels.setBrightness( 20 );

	// Set the LED color to gray to indicate setup is underway.
	pixels.fill( GRAY );
	pixels.show();

	Serial.println( "Setup is initializing the I2C bus for the Stemma QT port." );
	Wire.setPins( SDA1, SCL1 );	// This is what selects the Stemma QT port, otherwise the two pin headers will be I2C.
	Wire.begin();

	Serial.println( __FILE__ );

	// Set up the sensor.
	setupBme688();

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	Serial.println( "Connecting WiFi..." );
	// Set the LED color to yellow.
	pixels.fill( YELLOW );
	pixels.show();

	// Located in NetworkFunctions.ino
	wifiMultiConnect();

	if( WiFi.status() != WL_CONNECTED )
	{
		Serial.println( "Connection Failed! Rebooting..." );
		// Set the LED color to red.
		pixels.fill( RED );
		pixels.show();
		delay( 5000 );
		// Tell the SDK to reboot.  Sometimes more reliable than ESP.reset().
		ESP.restart();
	}

	// The networkIndex variable is initialized to 2112.  If it is still 2112 at this point, then WiFi failed to connect.
	if( networkIndex != 2112 )
	{
		const char *mqttBroker = mqttBrokerArray[networkIndex];
		const int mqttPort = mqttPortArray[networkIndex];
		// Set the MQTT client parameters.
		mqttClient.setServer( mqttBroker, mqttPort );
		// Assign the onReceiveCallback() function to handle MQTT callbacks.
		mqttClient.setCallback( onReceiveCallback );
		Serial.printf( "Using MQTT broker: '%s:%d\n", mqttBroker, mqttPort );
	}

	// Set the LED color to yellow.
	pixels.fill( YELLOW );
	pixels.show();

	// Located in NetworkFunctions.ino
	configureOTA();

	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.printf( "IP address: %s\n", ipAddress );

	// Set the LED color to green.
	pixels.fill( GREEN );
	pixels.show();

	Serial.println( "Setup is complete!" );
} // End of setup() function.


void setupBme688()
{
	Serial.println( "Initializing the BME688 sensor..." );
	if( !bme.begin() )
	{
		while( 1 )
		{
			Serial.println( "Could not find a valid BME688 sensor, check wiring!" );
			// Set the LED color to red and wait one second.
			pixels.fill( RED );
			pixels.show();
			delay( 1000 );
			// Set the LED color to yellow and wait a half second.
			pixels.fill( YELLOW );
			pixels.show();
			delay( 500 );
		}
	}

	// Set up oversampling and filter initialization
	bme.setTemperatureOversampling( BME680_OS_8X );
	bme.setHumidityOversampling( BME680_OS_2X );
	bme.setPressureOversampling( BME680_OS_4X );
	bme.setIIRFilterSize( BME680_FILTER_SIZE_3 );
	bme.setGasHeater( 320, 150 );		// 320*C for 150 ms

	Serial.println( "BME688 has been initialized." );
} // End of setupBme688() function.


int readTelemetry()
{
	rssi = WiFi.RSSI();
	if( !bme.performReading() )
		return 1;

	tempC = bme.temperature;
	tempF = ( tempC * 9 / 5 ) + 32;
	pressureHPa = bme.pressure / 100.0;
	humidity = bme.humidity;
	gasResistance = bme.gas_resistance / 1000.0;
	altitudeM = bme.readAltitude( SEALEVELPRESSURE_HPA );
	return 0;
} // End of readTelemetry() function.


/*
 * printUptime() will print the uptime to the serial port.
 */
void printUptime()
{
	Serial.print( "Uptime in " );
	long seconds = ( millis() - bootTime ) / 1000;
	long minutes = seconds / 60;
	long hours = minutes / 60;
	if( seconds < 601 )
	{
		Serial.print( "seconds: " );
		Serial.println( seconds );
	}
	else if( minutes < 121 )
	{
		Serial.print( "minutes: " );
		Serial.println( minutes );
	}
	else
	{
		Serial.print( "hours: " );
		Serial.println( hours );
	}
} // End of printUptime() function.


/*
 * printTelemetry() will print the sensor and device data to the serial port.
 */
void printTelemetry()
{
	Serial.printf( "WiFi SSID: %s\n", wifiSsidArray[networkIndex] );
	Serial.printf( "Broker: %s:%d\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex] );
	Serial.printf( "Temperature: %.2f C\n", tempC );
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Humidity: %.2f %%\n", humidity );
	Serial.printf( "Pressure = %.2f hPa\n", pressureHPa );
	Serial.printf( "Gas: %.2f KOhms\n", gasResistance );
	Serial.printf( "Approximate altitude: %.2f m\n", altitudeM );
	Serial.printf( "WiFi RSSI: %ld\n", rssi );
} // End of printTelemetry() function.


/*
 * publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	// Create a JSON Document on the stack.
	StaticJsonDocument<JSON_DOC_SIZE> publishTelemetryJsonDoc;
	// Add data: __FILE__, macAddress, ipAddress, tempC, pressureHPa, humidity, gasResistance, altitudeM, rssi, publishCount, notes
	publishTelemetryJsonDoc["sketch"] = __FILE__;
	publishTelemetryJsonDoc["mac"] = macAddress;
	publishTelemetryJsonDoc["ip"] = ipAddress;
	publishTelemetryJsonDoc["rssi"] = rssi;
	publishTelemetryJsonDoc["publishCount"] = publishCount;
	publishTelemetryJsonDoc["notes"] = notes;

	publishTelemetryJsonDoc["tempC"] = tempC;
	publishTelemetryJsonDoc["tempF"] = tempF;
	publishTelemetryJsonDoc["pressure"] = pressureHPa;
	publishTelemetryJsonDoc["humidity"] = humidity;
	publishTelemetryJsonDoc["gasResistance"] = gasResistance;
	publishTelemetryJsonDoc["altitudeM"] = altitudeM;

	// Prepare a String to hold the JSON.
	char mqttString[JSON_DOC_SIZE];

	// Serialize the JSON into mqttString, with indentation and line breaks.
	serializeJsonPretty( publishTelemetryJsonDoc, mqttString );

	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		Serial.println( "Successfully published to:" );
		char buffer[20];
		// New topic format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( sketchTopic, __FILE__, false ) )
			Serial.printf( "  %s\n", sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.printf( "  %s\n", macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.printf( "  %s\n", ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", rssiTopic );
		if( mqttClient.publish( publishCountTopic, ltoa( publishCount, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", publishCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.printf( "  %s\n", notesTopic );

		dtostrf( tempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempCTopic );
		dtostrf( tempF, 1, 3, buffer );
		if( mqttClient.publish( tempFTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempFTopic );
		dtostrf( pressureHPa, 1, 3, buffer );
		if( mqttClient.publish( pressureHPaTopic, buffer, false ) )
			Serial.printf( "  %s\n", pressureHPaTopic );
		dtostrf( humidity, 1, 3, buffer );
		if( mqttClient.publish( humidityTopic, buffer, false ) )
			Serial.printf( "  %s\n", humidityTopic );
		dtostrf( gasResistance, 1, 3, buffer );
		if( mqttClient.publish( gasResistanceTopic, buffer, false ) )
			Serial.printf( "  %s\n", gasResistanceTopic );
		dtostrf( altitudeM, 1, 3, buffer );
		if( mqttClient.publish( altitudeMTopic, buffer, false ) )
			Serial.printf( "  %s\n", altitudeMTopic );

		Serial.printf( "Successfully published to '%s', this JSON:\n", mqttTopic );
	}
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this JSON to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();
} // End of publishTelemetry() function.


/*
 * publishStats() is called by mqttMultiConnect() every time the device (re)connects to the broker, and every publishInterval milliseconds thereafter.
 * It is also called by the callback when the "publishStats" command is received.
 */
void publishStats()
{
	char mqttStatsString[JSON_DOC_SIZE];
	// Create a JSON Document on the stack.
	StaticJsonDocument<JSON_DOC_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, rssi, publishCount
	doc["sketch"] = __FILE__;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["rssi"] = rssi;
	doc["publishCount"] = publishCount;

	// Serialize the JSON into mqttStatsString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttStatsString );

	Serial.printf( "Publishing stats to the '%s' topic.\n", mqttStatsTopic );

	if( mqttClient.connected() )
	{
		if( mqttClient.connected() && mqttClient.publish( mqttStatsTopic, mqttStatsString ) )
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex], mqttStatsTopic, mqttStatsString );
		else
			Serial.println( "\n\nPublish failed!\n\n" );
	}
} // End of publishStats() function.


void loop()
{
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
		mqttMultiConnect( 5 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	ArduinoOTA.handle();
	yield();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( ( time > sensorPollInterval ) && ( time - sensorPollInterval ) > lastPollTime ) )
	{
		readTelemetry();
		lastPollTime = millis();
		Serial.print( "Next telemetry poll in " );
		Serial.print( sensorPollInterval / 1000 );
		Serial.println( " seconds.\n" );
	}

	time = millis();
	if( ( time > publishInterval ) && ( time - publishInterval ) > lastPublishTime )
	{
		publishCount++;
		Serial.println();
		Serial.println( __FILE__ );

		// Populate tempC and humidity objects with fresh data.
		readTelemetry();
		printUptime();
		printTelemetry();
		publishTelemetry();
		publishStats();

		Serial.printf( "publishCount: %lu\n", publishCount );

		lastPublishTime = millis();
		Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / 1000 );
	}
} // End of loop() function.
