/**
 * This sketch is a branch of my PubSubWeather sketch.
 * This sketch will use a BME688 sensor to show temperature, pressure, humidity, and gas readings.
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include "WiFi.h"						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "Adafruit_BME680.h"		// Adafruit BME680 (and BME688) library.  https://github.com/adafruit/Adafruit_BME680
#include <ESPmDNS.h>					// OTA
#include <WiFiUdp.h>					// OTA
#include <ArduinoOTA.h>				// OTA
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <Adafruit_NeoPixel.h>	// The Adafruit NeoPixel library to drive the RGB LED on the QT Py.	https://github.com/adafruit/Adafruit_NeoPixel
#include <ArduinoJson.h>			// https://arduinojson.org/


// NeoPixel related values.
#define NUMPIXELS        1
#define RED 0xFF0000
#define ORANGE 0xFFA500
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define INDIGO 0x4B0082
#define VIOLET 0xEE82EE
#define PURPLE 0x800080
#define BLACK 0x000000
#define GRAY 0x808080
#define WHITE 0xFFFFFF


/**
 * Global variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * I store those variables in "privateInfo.h", which is not uploaded to GitHub.
 */
//const char *wifiSsid = "yourSSID";				// Typically kept in "privateInfo.h".
//const char *wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char *mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;							// Typically kept in "privateInfo.h".
const char * sketchName = "QTPy_ESP32_BME688_OTA";
const char * notes = "Adafruit QT Py ESP32-S2 with BME688 and OTA";
// My topic format is: location/<device name>/sensor/<temperature, humidity, pressure, etc.>
const char * commandTopic = "livingRoom/QTPy/command";							// The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char * sketchTopic = "livingRoom/QTPy/sketch";								// The topic used to publish the sketch name.
const char * macTopic = "livingRoom/QTPy/mac";										// The topic used to publish the MAC address.
const char * ipTopic = "livingRoom/QTPy/ip";											// The topic used to publish the IP address.
const char * rssiTopic = "livingRoom/QTPy/rssi";									// The topic used to publish the WiFi Received Signal Strength Indicator.
const char * loopCountTopic = "livingRoom/QTPy/loopCount";						// The topic used to publish the loop count.
const char * notesTopic = "livingRoom/QTPy/notes";									// The topic used to publish notes relevant to this project.
const char * tempCTopic = "livingRoom/QTPy/bme688/tempC";						// The topic used to publish the temperature.
const char * pressureHPaTopic = "livingRoom/QTPy/bme688/pressureHPa";		// The topic used to publish the barometric pressure in hectoPascals/millibars.
const char * humidityTopic = "livingRoom/QTPy/bme688/humidity";				// The topic used to publish the humidity.
const char * gasResistanceTopic = "livingRoom/QTPy/bme688/gasResistance";	// The topic used to publish the gas resistance.
const char * altitudeTopic = "livingRoom/QTPy/bme688/altitude";				// The topic used to publish the altitude (derived from barometric pressure).
const char * mqttTopic = "espWeather";													// The topic used to publish a single JSON message containing all data.
unsigned long lastPublishTime = 0;														// This is used to determine the time since last MQTT publish.
unsigned int consecutiveBadTemp = 0;													//
unsigned int consecutiveBadHumidity = 0;												//
unsigned long sensorPollDelay = 10000;													// This is the delay between polls of the soil sensor.  This should be greater than 100 milliseconds.
unsigned long lastPollTime = 0;															// This is used to determine the time since last sensor poll.
float SEALEVELPRESSURE_HPA = 1025.0;													// This is the sea-level barometric pressure for the sensor's location.
char ipAddress[16];
char macAddress[18];
int loopCount = 0;
int publishDelay = 60000;
uint32_t start;
uint32_t stop;
unsigned long lastPublish = 0;							// In milliseconds, this sets a limit at 49.7 days of time.
unsigned long mqttReconnectDelay = 5000;				// An unsigned long can hold values from 0-4,294,967,295.  In milliseconds, this sets a limit at 49.7 days of time.
float bmeTempC;
float bmePressureHPa;
float bmeHumidity;
float bmeGasResistance;
float bmeAltitude;


// Class objects.
WiFiClient espClient;																			// Network client.  Used only byt the MQTT client.
PubSubClient mqttClient( espClient );														// MQTT client to manage communication to the broker.
Adafruit_NeoPixel pixels( NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800 );		// An object to manage the onboard RGB LED.
Adafruit_BME680 bme;																				// The Bosche BME688 atmospheric sensor.


void onReceiveCallback( char * topic, byte * payload, unsigned int length )
{
	char str[length + 1];
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	int i=0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char ) payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	// Add the null terminator.
	str[i] = 0;
	StaticJsonDocument <256> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, or publishStatus.
	const char * command = doc["command"];
	if( strcmp( command, "publishTelemetry") == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval") == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishDelay = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishDelay );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStatus") == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
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

	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Serial.println( "Setup is initializing the I2C bus for the Stemma QT port." );
	Wire.setPins( SDA1, SCL1 );	// This is what selects the Stemma QT port, otherwise the two pin headers will be I2C.
	Wire.begin();

	Serial.println( __FILE__ );

	// Set up the sensor.
	setupBme688();

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	Serial.println( "Connecting WiFi..." );
	// Set the LED color to yellow.
	pixels.fill( YELLOW );
	pixels.show();

	WiFi.mode( WIFI_STA );
	WiFi.begin( wifiSsid, wifiPassword );
	while( WiFi.waitForConnectResult() != WL_CONNECTED )
	{
		Serial.println( "Connection Failed! Rebooting..." );
		// Set the LED color to red.
		pixels.fill( RED );
		pixels.show();
		delay( 5000 );
		// Tell the SDK to reboot.  Sometimes more reliable than ESP.reset().
		ESP.restart();
	}

	// Port defaults to 3232
	// ArduinoOTA.setPort( 3232 );

	// Hostname defaults to esp32-[MAC]
	// ArduinoOTA.setHostname( "myesp32" );

	// No authentication by default
	// ArduinoOTA.setPassword( "admin" );

	// Password can be set with it's md5 value as well
	// MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );

	ArduinoOTA
		.onStart( []()
		{
			String type;
			if( ArduinoOTA.getCommand() == U_FLASH )
				type = "sketch";
			else // U_SPIFFS
				type = "filesystem";

			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println( "Start updating " + type );
		} )
		.onEnd( []()
		{
			Serial.println( "\nEnd" );
		} )
		.onProgress( []( unsigned int progress, unsigned int total )
		{
			Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) );
		} )
		.onError( []( ota_error_t error )
		{
			Serial.printf( "Error[%u]: ", error );
			if( error == OTA_AUTH_ERROR )
				Serial.println( "Auth Failed" );
			else if( error == OTA_BEGIN_ERROR )
				Serial.println( "Begin Failed" );
			else if( error == OTA_CONNECT_ERROR )
				Serial.println( "Connect Failed" );
			else if( error == OTA_RECEIVE_ERROR )
				Serial.println( "Receive Failed" );
			else if( error == OTA_END_ERROR )
				Serial.println( "End Failed" );
		} );

	ArduinoOTA.begin();

	Serial.println( "Ready" );
	Serial.print( "IP address: " );
	Serial.println( WiFi.localIP() );

	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );

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


void readTelemetry()
{
	if( !bme.performReading() )
	{
		Serial.println( "Failed to perform reading :(" );
		return;
	}

	bmeTempC = bme.temperature;
	bmePressureHPa = bme.pressure / 100.0;
	bmeHumidity = bme.humidity;
	bmeGasResistance = bme.gas_resistance / 1000.0;
	bmeAltitude = bme.readAltitude( SEALEVELPRESSURE_HPA );

	Serial.print( "Temperature = " );
	Serial.print( bmeTempC );
	Serial.println( " *C" );

	Serial.print( "Pressure = " );
	Serial.print( bmePressureHPa );
	Serial.println( " hPa" );

	Serial.print( "Humidity = " );
	Serial.print( bmeHumidity );
	Serial.println( " %" );

	Serial.print( "Gas = " );
	Serial.print( bmeGasResistance );
	Serial.println( " KOhms" );

	Serial.print( "Approx. Altitude = " );
	Serial.print( bmeAltitude );
	Serial.println( " m" );

	Serial.println();
} // End of readTelemetry() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
bool mqttConnect( int maxAttempts )
{
	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Set the LED color to orange while we reconnect.
		pixels.fill( ORANGE );
		pixels.show();

		char clientId[22];
		// Put the macAddress and a random number into clientId.  The random number suffix prevents brokers from rejecting a clientID as already in use.
		snprintf( clientId, 22, "%s-%03d", macAddress, random( 999 ) );
		Serial.print( "Connecting with client ID '" );
		Serial.print( clientId );
		Serial.print( "' " );

		// Connect to the broker using the pseudo-random clientId.
		if( mqttClient.connect( clientId ) )
		{
			Serial.println( "connected!" );
			// Set the LED color to green.
			pixels.fill( GREEN );
			pixels.show();
		}
		else
		{
			int mqttState = mqttClient.state();
			/*
				Possible values for client.state():
				#define MQTT_CONNECTION_TIMEOUT     -4		// Note: This also comes up when the clientID is already in use.
				#define MQTT_CONNECTION_LOST        -3
				#define MQTT_CONNECT_FAILED         -2
				#define MQTT_DISCONNECTED           -1
				#define MQTT_CONNECTED               0
				#define MQTT_CONNECT_BAD_PROTOCOL    1
				#define MQTT_CONNECT_BAD_CLIENT_ID   2
				#define MQTT_CONNECT_UNAVAILABLE     3
				#define MQTT_CONNECT_BAD_CREDENTIALS 4
				#define MQTT_CONNECT_UNAUTHORIZED    5
			*/
			Serial.print( " failed!  Return code: " );
			Serial.print( mqttState );
			if( mqttState == -4 )
			{
				Serial.println( " - MQTT_CONNECTION_TIMEOUT" );
			}
			else if( mqttState == 2 )
			{
				Serial.println( " - MQTT_CONNECT_BAD_CLIENT_ID" );
			}
			else
			{
				Serial.println( "" );
			}

			Serial.print( "Trying again in " );
			Serial.print( mqttReconnectDelay / 1000 );
			Serial.println( " seconds." );
			delay( mqttReconnectDelay );
		}
		i++;
	}
	if( mqttClient.connected() )
	{
		// Subscribe to backYard/Lolin8266/command, which will respond to publishTelemetry and publishStatus
		mqttClient.subscribe( commandTopic );
		mqttClient.setBufferSize( 512 );
		char connectString[512];
		snprintf( connectString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", sketchName, macAddress, ipAddress );
		mqttClient.publish( "espConnect", connectString );
		// Set the LED color to green.
		pixels.fill( GREEN );
		pixels.show();
	}
	else
	{
		Serial.println( "Unable to connect to the MQTT broker!" );
		return false;
	}

	Serial.println( "Function mqttConnect() has completed." );
	return true;
} // End of mqttConnect() function.


/*
 * publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "WiFi RSSI: " );
	Serial.println( rssi );
	// Prepare a String to hold the JSON.
	char mqttString[512];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"pressureHPa\": %.2f,\n\t\"humidity\": %.2f,\n\t\"gas\": %.2f,\n\t\"altitudeM\": %.2f,\n\t\"rssi\": %ld,\n\t\"loopCount\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, bmeTempC, bmePressureHPa, bmeHumidity, bmeGasResistance, bmeAltitude, rssi, loopCount, notes );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		Serial.println( "Successfully published to:" );
		char buffer[20];
		// New topic format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( sketchTopic, sketchName, false ) )
			Serial.println( sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.println( macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.println( ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
			Serial.println( rssiTopic );
		if( mqttClient.publish( loopCountTopic, ltoa( loopCount, buffer, 10 ), false ) )
			Serial.println( loopCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.println( notesTopic );

		dtostrf( bmeTempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.println( tempCTopic );
		if( mqttClient.publish( pressureHPaTopic, ltoa( bmePressureHPa, buffer, 10 ), false ) )
			Serial.println( pressureHPaTopic );
		if( mqttClient.publish( humidityTopic, ltoa( bmeHumidity, buffer, 10 ), false ) )
			Serial.println( humidityTopic );
		if( mqttClient.publish( gasResistanceTopic, ltoa( bmeGasResistance, buffer, 10 ), false ) )
			Serial.println( gasResistanceTopic );
		if( mqttClient.publish( altitudeTopic, ltoa( bmeAltitude, buffer, 10 ), false ) )
			Serial.println( altitudeTopic );

		Serial.print( "Successfully published to '" );
		Serial.print( mqttTopic );
		Serial.println( "', this JSON:" );
	}
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this JSON to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();
} // End of publishTelemetry() function.


void loop()
{
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
		mqttConnect( 10 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	ArduinoOTA.handle();
	yield();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( ( time > sensorPollDelay ) && ( time - sensorPollDelay ) > lastPollTime ) )
	{
		readTelemetry();
		lastPollTime = millis();
		Serial.print( "Next telemetry poll in " );
		Serial.print( sensorPollDelay / 1000 );
		Serial.println( " seconds.\n" );
	}

	time = millis();
	if( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish )
	{
		loopCount++;
		Serial.println();
		Serial.println( sketchName );
		Serial.println( __FILE__ );

		// Populate tempC and humidity objects with fresh data.
		readTelemetry();

		publishTelemetry();

		Serial.print( "loopCount: " );
		Serial.println( loopCount );

		lastPublish = millis();
		Serial.print( "Next publish in " );
		Serial.print( publishDelay / 1000 );
		Serial.println( " seconds.\n" );
	}
} // End of loop() function.
