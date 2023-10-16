/* Code for hardware module of AMFDC-D & Fire Controller - Meteorological & Geospatial only - Arduino Nano based
	Transmits BME280 and GNSS data over BlueTooth (HC-05)
	Author: Deep Pradhan, 2021

	Wiring:
		BME280 SDA  <-->  Arduino Uno/Nano A4
		BME280 SCL  <-->  Arduino Uno/Nano A5
		GNSS RXD    <---  Arduino Uno/Nano D2 (Software Serial TX)
		GNSS TXD    --->  Arduino Uno/Nano D3 (Software Serial RX)
		HC-05 RX    <---  Arduino Uno/Nano D0/TX (via voltage divider)
		HC-05 TX    --->  Arduino Uno/Nano D1/RX */

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <Wire.h>

#define size_array(x) (sizeof(x) / sizeof(x[0]))

// GNSS Pins
#define PIN_SOFTWARE_SERIAL_RX 3 // Software Serial RX
#define PIN_SOFTWARE_SERIAL_TX 2 // Software Serial TX

// Interval of meteorological data update: 500ms
#define INTERVAL_METEOROLOGICAL_UPDATE 500

// Default interval of transmission: 10 seconds
#define INTERVAL_TRANSMIT 5 * 1000

// Sea level standard atmospheric pressure
#define PRESSURE_SEA_LEVEL_HPA (1013.25)

#define PRECISION_DEFAULT 1

// Decimal point precision for geodetic coordinates (longitude/latitude)
#define PRECISION_GEODETIC 6

// Format of message of Meteorological & Geospatial data
/* $MTGS,<Temperature (celsius)>,<Pressure (hPa)>,<Humidity (%)>,
			<Barometric altitude (m)>,<longitude (decimal degrees)>,
			<latitude (decimal degrees)>,<altitude (m, GNSS)>,<seconds since UNIX epoch>,
			<DOP>,<satellites in use>,<satellites visible>,<constellations available flag>*/
#define FORMAT_MESSAGE_METEOROLOGICAL_GEOSPATIAL "$MTGS,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"

#define STR_NULL "\0"

// Connected with BME280
bool connected_bme280 = true;

// Connected with GNSS
bool connected_gnss = false;

// Has a GNSS fix being achieved
bool gnss_location_available = false;

// Time of last meteorological data update in milliseconds
unsigned long time_last_meteorological = 0;

// Time of last transmission in milliseconds
unsigned long time_last_transmit = 0;

unsigned int interval_transmit = INTERVAL_TRANSMIT;

unsigned int _satellites_visible;

unsigned int _gnss_constellations_available;

char temperature[6] = STR_NULL,
		pressure[8] = STR_NULL,
		humidity[7] = STR_NULL,
		altitude_barometric[7] = STR_NULL,
		longitude[PRECISION_GEODETIC + 6] = STR_NULL,
		latitude[PRECISION_GEODETIC + 5] = STR_NULL,
		altitude_gnss[7] = STR_NULL,
		gnss_time[11] = STR_NULL,
		dop[6] = STR_NULL,
		satellites_in_use[4] = STR_NULL,
		satellites_visible[4] = STR_NULL,
		gnss_constellations_available[3] = STR_NULL,
		buffer_metorological_geospatial[84];

// Time struct
tmElements_t date_time;

// Create a BME280 object
Adafruit_BME280 bme280;

// Create our Software Serial object
SoftwareSerial serial_gnss(PIN_SOFTWARE_SERIAL_RX, PIN_SOFTWARE_SERIAL_TX);

// Create a TinyGPS++ object
TinyGPSPlus gnss;

TinyGPSCustom custom_fields_satellites_visible[] = {
		TinyGPSCustom(gnss, "GPGSV", 3), // GPS/Navstar (01)
		TinyGPSCustom(gnss, "GLGSV", 3), // GLONASS (02)
		TinyGPSCustom(gnss, "GNGSV", 3), // GLONASS (Multi-constilation) (02)
		TinyGPSCustom(gnss, "GBGSV", 3), // BeiDou (04)
		TinyGPSCustom(gnss, "GAGSV", 3), // Galileo (08)
		TinyGPSCustom(gnss, "GIGSV", 3) // IRNSS/NavIC (16)
};

const int SIZE_GNSS_VISIBLE = size_array(custom_fields_satellites_visible);

void setup() {
	Serial.begin(9600);
	serial_gnss.begin(9600);
	Serial.println("Smart Hub Prototype (Metrological & Geospatial only)\n(c) Deep Pradhan 2022");
 
	if (!bme280.begin(0x77) && !bme280.begin(0x76)) // Default address '0x77', some may have set as '0x76'
		connected_bme280 = false;
}

void loop() {
	if (connected_bme280 && time_last_meteorological + INTERVAL_METEOROLOGICAL_UPDATE <= millis()) {
		time_last_meteorological = millis(); // Save time of last meteorological for timing
		// Read BME280 data and update char arrays
		dtostrf(bme280.readTemperature(), 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, temperature);
		dtostrf(bme280.readPressure() / 100.0F, 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, pressure);
		dtostrf(bme280.readHumidity(), 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, humidity);
		dtostrf(bme280.readAltitude(PRESSURE_SEA_LEVEL_HPA), 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, altitude_barometric);
	}

	_satellites_visible = 0;
	_gnss_constellations_available = 0;

	// Read & process GNSS data
	while (serial_gnss.available()) {
		gnss.encode(serial_gnss.read());
		connected_gnss = true;
	}
	// Location (longitude, latitude, altitude)
	if (gnss.location.isValid() && gnss.location.isUpdated()) { // Valid location received - update char arrays
		dtostrf(gnss.location.lng(), 2 + PRECISION_GEODETIC, PRECISION_GEODETIC, longitude);
		dtostrf(gnss.location.lat(), 2 + PRECISION_GEODETIC, PRECISION_GEODETIC, latitude);
		if (gnss.altitude.isValid())
			dtostrf(gnss.altitude.meters(), 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, altitude_gnss);
		else
			strcpy(altitude_gnss, STR_NULL);
		gnss_location_available = true;
	}
	// Time as per Unix Epoch (time)
	if (gnss.date.isValid() && gnss.time.isValid() && gnss.date.isUpdated() && gnss.time.isUpdated()) {
		date_time.Year = gnss.date.year() - 1970;
		date_time.Month = gnss.date.month();
		date_time.Day = gnss.date.day();
		date_time.Hour = gnss.time.hour();
		date_time.Minute = gnss.time.minute();
		date_time.Second = gnss.time.second();
		sprintf(gnss_time, "%lu", (unsigned long) makeTime(date_time)); // Get time with reference to Unix epoch
	}
	/* DOP: https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)#Interpretation
		<01: Ideal, 01-02: Excellent, 02-05: Good, 05-10: Moderate, 10-20: Fair, >20: Poor*/
	if (gnss.hdop.isValid() && gnss.hdop.isUpdated()) {
		dtostrf(gnss.hdop.hdop(), 2 + PRECISION_DEFAULT, PRECISION_DEFAULT, dop);
		if (gnss.hdop.hdop() <= 0 || gnss.hdop.hdop() >= 100) { // Location not available if DOP is 0 / 100
			strcpy(dop, STR_NULL);
			gnss_location_available = false;
		}
	}
	// Satellites in use
	if (gnss.satellites.isValid() && gnss.satellites.isUpdated()) {
		sprintf(satellites_in_use, "%d", gnss.satellites.value());
		if (gnss.satellites.value() == 0) // Location not available if satellites are 0
			gnss_location_available = false;
	}
	// Satellites visible
	for (int i = 0, got_glonass = 0, visible; i < SIZE_GNSS_VISIBLE; i++) {
		visible = atoi(custom_fields_satellites_visible[i].value());
		if (i == 1 && visible > 0) // GLONASS listed under GLGSV
			got_glonass = 1;
		if (i == 2 && got_glonass == 1) // Skip GNGSV as GLONASS already listed under GLGSV
			continue;
		_satellites_visible += visible;
		if (visible > 0)
      _gnss_constellations_available |= (unsigned int) pow(2, i);
	}
	sprintf(satellites_visible, "%d", _satellites_visible);
	sprintf(gnss_constellations_available, "%d", _gnss_constellations_available);

	if (!gnss_location_available) { // Fill char arrays with NULL if no GNSS fix
		strcpy(longitude, STR_NULL);
		strcpy(latitude, STR_NULL);
		strcpy(altitude_gnss, STR_NULL);
	}

	if (time_last_transmit + interval_transmit <= millis()) // Transmit after set interval
		transmit();
}

/** Function to transmit data over Bluetooth*/
void transmit() {
	time_last_transmit = millis(); // Save time of transmission for timing
	sprintf(buffer_metorological_geospatial, FORMAT_MESSAGE_METEOROLOGICAL_GEOSPATIAL, temperature, pressure, humidity, altitude_barometric,
			longitude, latitude, altitude_gnss, gnss_time, dop, satellites_in_use, satellites_visible, gnss_constellations_available);
	Serial.print(buffer_metorological_geospatial);
}
