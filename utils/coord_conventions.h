/**
 * Derived from MAV'RIC project
 */



#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "quaternions.h"
#include "small_matrix.h"
#include "constants.h"

#include <stdint.h>

#define EARTH_RADIUS 6378137.0f   // radius of the earth in meters

#define rad_to_deg(input) (input*180.0f/PI)
#define deg_to_rad(input) (input*PI/180.0f)


typedef struct 
{
	double longitude;			///<	Current longitude
	double latitude;			///<	Current latitude
	float altitude;				///<	Current altitude
	float heading;				///<	Current heading
	uint32_t timestamp_ms;		///<	Timestamp (milliseconds)
} global_position_t;


typedef struct 
{
	vector_3_t pos;				///<	Current position x, y and z
	float heading;				///<	Current heading (equal to heading in global frame)
	global_position_t origin;	///<	Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
	uint32_t timestamp_ms;		///<	Timestamp (milliseconds)
} local_coordinates_t;


typedef struct 
{
	float rpy[3];	///<	Roll pitch and yaw angles in radians
} aero_attitude_t;


global_position_t coord_conventions_local_to_global_position(local_coordinates_t input);


vector_3_t coord_conventions_get_vector(global_position_t pos1, global_position_t pos2);

local_coordinates_t coord_conventions_global_to_local_position(global_position_t position, global_position_t origin);

float coord_conventions_global_distance_2D(global_position_t pos1, global_position_t pos2);

float coord_conventions_global_distance_3D(global_position_t pos1, global_position_t pos2);

aero_attitude_t coord_conventions_quat_to_aero(quat_t qe);


quat_t coord_conventions_quaternion_from_aero(aero_attitude_t aero);


float coord_conventions_get_yaw(quat_t qe);

static inline float coord_conventions_get_roll(quat_t qe) 
{
	return  atan2(2*(qe.s*qe.v[0] + qe.v[1]*qe.v[2]) , (qe.s*qe.s - qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] + qe.v[2]*qe.v[2])); 
}

#ifdef __cplusplus
}
#endif

#endif /* COORD_CONVENTIONS_H_ */