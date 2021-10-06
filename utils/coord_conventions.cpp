
#include "coord_conventions.h"
#include "maths.h"

#include <math.h>

global_position_t coord_conventions_local_to_global_position(local_coordinates_t input) {
  global_position_t output;

  output.latitude = input.origin.latitude + rad_to_deg(input.pos.v[0] / EARTH_RADIUS);
  output.longitude = input.origin.longitude + rad_to_deg(input.pos.v[1] / (EARTH_RADIUS * cos(deg_to_rad(output.latitude))));
  output.altitude = -input.pos.v[2] + input.origin.altitude;
  output.heading = input.heading;
  output.timestamp_ms = input.timestamp_ms;

  return output;
}

vector_3_t coord_conventions_get_vector(global_position_t pos1, global_position_t pos2) {
  vector_3_t pos;
  double small_radius = cos(deg_to_rad(pos2.latitude)) * EARTH_RADIUS;
  pos.v[X] = (float) (sin(deg_to_rad((pos2.latitude - pos1.latitude))) * EARTH_RADIUS);
  pos.v[Y] = (float) (sin(deg_to_rad((pos2.longitude - pos1.longitude))) * small_radius);
  pos.v[Z] = (float) (-(pos2.altitude - pos1.altitude));
  return pos;
}

local_coordinates_t coord_conventions_global_to_local_position(global_position_t position, global_position_t origin) {
  local_coordinates_t output;
  output.pos = coord_conventions_get_vector(origin, position);
  output.origin = origin;
  output.heading = position.heading;
  output.timestamp_ms = position.timestamp_ms;

  return output;
}

float coord_conventions_global_distance_2D(global_position_t pos1, global_position_t pos2) {

  vector_3_t rel_pos = coord_conventions_get_vector(pos1, pos2);
  rel_pos.v[Z] = 0;
  return sqrt(sqr_norm3(rel_pos));
}

float coord_conventions_global_distance_3D(global_position_t pos1, global_position_t pos2) {

  vector_3_t rel_pos = coord_conventions_get_vector(pos1, pos2);
  return sqrt(sqr_norm3(rel_pos));
}

aero_attitude_t coord_conventions_quat_to_aero(quat_t qe) {
  aero_attitude_t aero;

  aero.rpy[0] = atan2(2 * (qe.s * qe.v[0] + qe.v[1] * qe.v[2]), (qe.s * qe.s - qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] + qe.v[2] * qe.v[2]));
  aero.rpy[1] = -asin(2 * (qe.v[0] * qe.v[2] - qe.s * qe.v[1]));
  aero.rpy[2] = atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]), (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));

  return aero;
}

quat_t coord_conventions_quaternion_from_aero(aero_attitude_t aero) {
  quat_t quat;

  // intermediate values
  float cr, cp, cy, sr, sp, sy;
  cr = cos(aero.rpy[0] / 2);
  cp = cos(aero.rpy[1] / 2);
  cy = cos(aero.rpy[2] / 2);
  sr = sin(aero.rpy[0] / 2);
  sp = sin(aero.rpy[1] / 2);
  sy = sin(aero.rpy[2] / 2);


  quat.s = (cr * cp * cy) + (sr * sp * sy);
  quat.v[0] = (sr * cp * cy) - (cr * sp * sy);
  quat.v[1] = (cr * sp * cy) + (sr * cp * sy);
  quat.v[2] = (cr * cp * sy) - (sr * sp * cy);

  return quat;
}

float coord_conventions_get_yaw(quat_t qe) {
  return atan2(2 * (qe.s * qe.v[2] + qe.v[0] * qe.v[1]), (qe.s * qe.s + qe.v[0] * qe.v[0] - qe.v[1] * qe.v[1] - qe.v[2] * qe.v[2]));
}
