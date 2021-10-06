

#include "helper_utils.h"
#include <iostream>

float h_distanceFromGps(global_position_t pos1, global_position_t pos2) {
    return (float)coord_conventions_global_distance_2D(pos1, pos2);
}

global_position_t h_vectToGps(Vector3 local_pos, global_position_t origin, double heading_deg) {
    local_coordinates_t local_position;
    local_position.pos.v[0] = local_pos(0);
    local_position.pos.v[1] = local_pos(1);
    local_position.pos.v[2] = local_pos(2);
    
    local_position.heading = heading_deg;
    local_position.origin = origin;
    
    global_position_t gpos = coord_conventions_local_to_global_position(local_position);
    return gpos;
}

Vector3 h_gpsToVect(global_position_t global_pos, global_position_t origin) {
    local_coordinates_t local_pos = coord_conventions_global_to_local_position(global_pos, origin);
    return Vector3((float)local_pos.pos.v[0],(float)local_pos.pos.v[1], (float)local_pos.pos.v[2]);
}