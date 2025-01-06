#include "NMEA_Parser.h"
#include <stdlib.h>

///////// Parser Functions /////////
GPS_GPGGA_DATA processGPGGA(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPGGA_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// Fix time
				gps_out.fix_time = (uint32_t)strtoul(aux_buff, NULL, 10);
				break;
			case 2:
				// Latitude
				gps_out.latitude = strtod(aux_buff, NULL);
				break;
			case 3:
				// Latitude direction
				gps_out.lat_direction = aux_buff[0];
				break;
			case 4:
				// Longitude
				gps_out.longitude = strtod(aux_buff, NULL);
				break;
			case 5:
				// Longitude direction
				gps_out.lon_direction = aux_buff[0];
				break;
			case 6:
				// Fix quality
				gps_out.fix_quality = (uint8_t)strtol(aux_buff, NULL, 10);
				break;
			case 7:
				// Satellite count
				gps_out.sat_count = (uint8_t)strtol(aux_buff, NULL, 10);
				break;
			case 8:
				// Horizontal dilution
				gps_out.horizontal_dilution = strtof(aux_buff, NULL);
				break;
			case 9:
				// Altitude
				gps_out.altitude = strtof(aux_buff, NULL);
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPRMC_DATA processGPRMC(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPRMC_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// Fix time
				gps_out.fix_time = (uint32_t)strtoul(aux_buff, NULL, 10);
				break;
			case 2:
				// Status code (A = Active, V = Void)
				gps_out.status = aux_buff[0];
				break;
			case 3:
				// Latitude
				gps_out.latitude = strtod(aux_buff, NULL);
				break;
			case 4:
				// Latitude direction
				gps_out.lat_direction = aux_buff[0];
				break;
			case 5:
				// Longitude
				gps_out.longitude = strtod(aux_buff, NULL);
				break;
			case 6:
				// Longitude direction
				gps_out.lon_direction = aux_buff[0];
				break;
			case 7:
				// Speed in knots
				gps_out.speed_knots = strtof(aux_buff, NULL);
				break;
			case 8:
				// Track angle
				gps_out.track_angle = strtof(aux_buff, NULL);
				break;
			case 10:
				// Magnetic variation degrees
				gps_out.mag_variation = strtof(aux_buff, NULL);
				break;
			case 11:
				// Magnetic variation direction
				gps_out.mag_variation_dir = aux_buff[0];
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPGLL_DATA processGPGLL(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPGLL_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// Latitude
				gps_out.latitude = strtod(aux_buff, NULL);
				break;
			case 2:
				// Latitude direction
				gps_out.lat_direction = aux_buff[0];
				break;
			case 3:
				// Latitude
				gps_out.longitude = strtod(aux_buff, NULL);
				break;
			case 4:
				// Longitude direction
				gps_out.lon_direction = aux_buff[0];
				break;
			case 5:
				// Status code (A = Active, V = Void)
				gps_out.status = aux_buff[0];
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPVTG_DATA processGPVTG(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPVTG_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// True track made good
				gps_out.true_track = strtof(aux_buff, NULL);
				break;
			case 2:
				// Magnetic track made good
				gps_out.mag_track = strtof(aux_buff, NULL);
				break;
			case 3:
				// Speed in knots
				gps_out.speed_knots = strtof(aux_buff, NULL);
				break;
			case 4:
				// Speed in km/h
				gps_out.speed_kmh = strtof(aux_buff, NULL);
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPGSA_DATA processGPGSA(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPGSA_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// Operation mode (A = Auto, M = Manual)
				gps_out.operation_mode = aux_buff[0];
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPGSV_DATA processGPGSV(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPGSV_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 2:
				// Sentence number
				gps_out.sentence_num = strtol(aux_buff, NULL, 10);
				break;
			case 3:
				// Number of satellites in view
				gps_out.total_sats = strtol(aux_buff, NULL, 10);
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

GPS_GPZDA_DATA processGPZDA(char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPZDA_DATA gps_out;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// UTC time
				gps_out.utc_time = (uint32_t)strtoul(aux_buff, NULL, 10);
				break;
			case 2:
				// Day
				gps_out.day = strtol(aux_buff, NULL, 10);
				break;
			case 3:
				// Month
				gps_out.month = strtol(aux_buff, NULL, 10);
				break;
			case 4:
				// Year
				gps_out.year = (uint16_t)strtoul(aux_buff, NULL, 10);
				break;
			default:
				break;
			}

			index = 0;
			item_count ++;
			memset(aux_buff, 0, sizeof(aux_buff));
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
	return gps_out;
}

// Parse and store all relevant data (Modify this to store relevant data for the application)
GPS_DATA parseGPSData(char* gps_buffer){
	GPS_DATA gps_out;
	// All NMEA sentences start with $GP(x_x_x), -> Use this to select parse mode
	char id[3] = {*(gps_buffer + 3), *(gps_buffer + 4), *(gps_buffer + 5)};
	// Compare id and parse accordingly
	if(strcmp(id, "GGA")){
		gps_out.GPGGA_data = processGPGGA(gps_buffer);
	}
	else if(strcmp(id, "RMC")){
		processGPRMC(gps_buffer);
	}
	else if(strcmp(id, "GLL")){
		processGPGLL(gps_buffer);
	}
	else if(strcmp(id, "VTG")){
		gps_out.GPVTG_data = processGPVTG(gps_buffer);
	}
	else if(strcmp(id, "GSA")){
		processGPGSA(gps_buffer);
	}
	else if(strcmp(id, "GSV")){
		gps_out.GPGSV_data = processGPGSV(gps_buffer);
	}
	else if(strcmp(id, "ZDA")){
		processGPZDA(gps_buffer);
	}
	return gps_out;
}
