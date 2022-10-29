cmake_minimum_required( VERSION 3.23 )
project( QTPy_ESP32_BME688_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( . )

add_executable( QTPy_ESP32_BME688_OTA
		NetworkFunctions.ino
		privateInfo.h
		QTPy_ESP32_BME688_OTA.ino )
