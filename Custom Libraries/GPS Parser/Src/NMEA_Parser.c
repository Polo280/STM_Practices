#include "NMEA_Parser.h"
#include <stdlib.h>

///////// Parser Functions /////////
GPS_GPGGA_DATA processGPGGA(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
	uint8_t index = 0;  // To store sequentially in aux buffer
	uint8_t item_count = 0;
	GPS_GPGGA_DATA gps_out;

	while(*c != '\r' && *c != '\n' && *c != '\0'){
		if(*c == ','){
			switch(item_count){
			case 1:
				// Fix time
				gps_out.fix_time = strtof(aux_buff, NULL);
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

void processGPRMC(const char* sentence, GPS_GPRMC_DATA* rmc_data){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
	// To store sequentially in aux buffer
	uint8_t index = 0;
	uint8_t item_count = 0;
	// Parsing auxiliaries
	char validator = '';
	uint32_t aux = 0;

	while(*c != '\n' && *c != '\0'){
		if(*c == ','){
			if(aux_buff[0] != '\0'){
				// Null terminate the buffer to cast
				aux_buff[index] = '\0';

				switch(item_count){
				case 1:
					// Fix time
					 aux = (uint32_t)strtoul(aux_buff, NULL, 10);
					// Filter out incomplete fix time
					if(aux > 100){
						rmc_data->fix_time = aux;
					}
					break;
				case 2:
					// Status code (A = Active, V = Void)
					validator = aux_buff[0];
					rmc_data->status = validator;
					break;
				case 3:
					// Latitude
					if(validator == 'A'){
						rmc_data->latitude = strtof(aux_buff, NULL);
					}
					break;
				case 4:
					// Latitude direction
					if(validator == 'A'){
						rmc_data->lat_direction = aux_buff[0];
					}
					break;
				case 5:
					// Longitude
					if(validator == 'A'){
						rmc_data->longitude = strtof(aux_buff, NULL);
					}
					break;
				case 6:
					// Longitude direction
					if(validator == 'A'){
						rmc_data->lon_direction = aux_buff[0];
					}
					break;
				case 7:
					// Speed in knots
					rmc_data->speed_knots = strtof(aux_buff, NULL);
					break;
				case 8:
					// Track angle
					rmc_data->track_angle = strtof(aux_buff, NULL);
					break;
				case 9:
					// Date
					aux = (uint32_t)strtoul(aux_buff, NULL, 10);
					// Filter out incomplete date
					if(aux > 100){
						rmc_data->date = aux;
					}
					break;
				case 10:
					// Magnetic variation degrees
					if(validator == 'A'){
						rmc_data->mag_variation = strtof(aux_buff, NULL);
					}
					break;
				case 11:
					// Magnetic variation direction
					if(validator == 'A'){
						rmc_data->mag_variation_dir = aux_buff[0];
					}
					break;
				default:
					break;
				}
				index = 0;
				memset(aux_buff, 0, sizeof(aux_buff));
			}
			item_count ++;
		}
		else{
			aux_buff[index] = *c;
			index ++;
		}
		c ++;
	}
}

GPS_GPGLL_DATA processGPGLL(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
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

GPS_GPVTG_DATA processGPVTG(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
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

GPS_GPGSA_DATA processGPGSA(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
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

GPS_GPGSV_DATA processGPGSV(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
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

GPS_GPZDA_DATA processGPZDA(const char* sentence){
	// Iterate through sentence bytes
	char aux_buff[25];
	const char* c = sentence;
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

// Split valid NMEA Sentences in a message
void splitNMEASentences(const char *buffer, char sentences[MAX_SENTENCES_SPLIT][MAX_SENTENCE_LENGTH]){
	uint8_t sentence_count = 0;
	// Read only pointers
	const char* start = buffer;
	const char* end;

	while((end = strstr(start, "\n")) != NULL){
		size_t length = end - start + 1;

		if (length < MAX_SENTENCE_LENGTH && sentence_count < MAX_SENTENCES_SPLIT) {
			strncpy(sentences[sentence_count], start, length);
			sentences[sentence_count][length] = '\0';  // Null-terminate the string
			sentence_count++;
		}
		start = end + 1;
	}
}

// Parse and store all relevant data (Modify this to store relevant data for the application)
void parseGPSData(const char* gps_buffer, GPS_DATA* gps_data){
	// All NMEA sentences start with $GP(x_x_x), -> Use this to select parse mode
	char id[3] = {*(gps_buffer + 4), *(gps_buffer + 5)};
	// Compare id and parse accordingly
//	if(strcmp(id, "GA")){  // GGA
//		gps_out.GPGGA_data = processGPGGA(gps_buffer);
//	}
	if(strcmp(id, "MC") == 0){ // RMC
		processGPRMC(gps_buffer, &gps_data->GPRMC_data);
	}
//	else if(strcmp(id, "LL")){ // GLL
//		processGPGLL(gps_buffer);
//	}
//	else if(strcmp(id, "TG")){ // VTG
//		gps_out.GPVTG_data = processGPVTG(gps_buffer);
//	}
//	else if(strcmp(id, "SA")){ // GSA
//		processGPGSA(gps_buffer);
//	}
//	else if(strcmp(id, "SV")){ // GSV
//		gps_out.GPGSV_data = processGPGSV(gps_buffer);
//	}
//	else if(strcmp(id, "DA")){ // ZDA
//		processGPZDA(gps_buffer);
//	}
}
