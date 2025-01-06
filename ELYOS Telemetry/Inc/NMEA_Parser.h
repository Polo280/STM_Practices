#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdio.h>
#include <string.h>
#include "main.h"

/////// NMEA SENTENCE TYPES ///////
// Global Positioning System Fix Data
typedef struct {
	uint32_t fix_time;
	uint8_t  fix_quality;
	uint8_t  sat_count;
	float 	 horizontal_dilution;
	float    altitude;
	double   latitude;
	double   longitude;
	char 	 lon_direction;
	char     lat_direction;
} GPS_GPGGA_DATA;

// Recomented Minimum Specific GPS / Transit Data
typedef struct {
	uint32_t fix_time;
	float 	 speed_knots;
	float 	 track_angle;
	float 	 mag_variation;
	double 	 latitude;
	double	 longitude;
	char 	 status;
	char 	 lat_direction;
	char 	 lon_direction;
	char 	 mag_variation_dir;
} GPS_GPRMC_DATA;

// Geographic Position (Latitude/Longitude)
typedef struct {
	uint32_t fix_time;
	double 	 latitude;
	double 	 longitude;
	char 	 status;
	char 	 lat_direction;
	char 	 lon_direction;
} GPS_GPGLL_DATA;

// Track made good and ground speed
typedef struct {
	float true_track;
	float mag_track;
	float speed_knots;
	float speed_kmh;
} GPS_GPVTG_DATA;

// GNSS DOP and active satellites
typedef struct {
	float pdop;
	float hdop;
	float vdop;
	char  operation_mode;
} GPS_GPGSA_DATA;

// GNSS Satellites in view
typedef struct {
	uint8_t sentence_num;
	uint8_t total_sats;
} GPS_GPGSV_DATA;

// Time and data
typedef struct {
	uint32_t utc_time;
	uint16_t year;
	uint8_t  day;
	uint8_t  month;
} GPS_GPZDA_DATA;


// Relevant data for this telemetry system
typedef struct {
	GPS_GPGGA_DATA GPGGA_data;
	GPS_GPVTG_DATA GPVTG_data;
	GPS_GPGSV_DATA GPGSV_data;
} GPS_DATA;

// Parsing functions
GPS_GPGGA_DATA processGPGGA(char*);
GPS_GPRMC_DATA processGPRMC(char*);
GPS_GPGLL_DATA processGPGLL(char*);
GPS_GPVTG_DATA processGPVTG(char*);
GPS_GPGSA_DATA processGPGSA(char*);
GPS_GPGSV_DATA processGPGSV(char*);
GPS_GPZDA_DATA processGPZDA(char*);
GPS_DATA parseGPSData(char*);

#endif /* NMEA_PARSER_H */
