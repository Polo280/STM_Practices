#ifndef COMP_FILTER_H
#define COMP_FILTER_H

//////////////////////////////////////////////////////
///// COMPLEMENTARY FILTER FOR VELOCITY ESTIMATE /////
//////////////////////////////////////////////////////

// MACROS
#define IMU_WEIGHT 0.5
#define GPS_WEIGHT 0.2

// Sensor weights
const float a1 = IMU_WEIGHT;
const float a2 = GPS_WEIGHT;
const float a3 = 1 - IMU_WEIGHT - GPS_WEIGHT;

// Filter state
typedef struct {

} CompFilterState;

#endif /* COMP_FILTER_H */
