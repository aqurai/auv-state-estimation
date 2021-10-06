/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "offline-processing.h"

#include <Eigen/Eigen>

#include "state_estimation.h"


uint8_t raw_data_logging_sysid = 91;
uint8_t processed_data_logging_sysid = 92;

void OfflineProcessing::openFiles(std::string fn_prefix) {
    state_stream.open(fn_prefix + "states.txt", std::ios::out);
    gps_stream.open(fn_prefix + "gps.txt", std::ios::out);
    gps_updates_stream.open(fn_prefix + "gps_updates.txt", std::ios::out);
    //aco_range_stream.open(fn_prefix + "aco_measurements.txt", std::ios::out);
}


void OfflineProcessing::sendProcessedRangeMeasurements(const StateEstimation* const se, int16_t timestamp_ms, uint8_t sysid, float range_m) {
#ifndef RPI_DBG
    mavlink_message_t msg;
    mavlink_msg_beacon_range_measurement_pack(processed_data_logging_sysid, se->mavlink_stream_->compid,
							&msg,
							timestamp_ms,
							sysid,
                                                        range_m,
                                                        (uint32_t)range_m * 1000000 / 1480u
							);
    mavlink_stream_send(se->mavlink_stream_, &msg);
#endif
}

void OfflineProcessing::sendProcessedBeaconPosition(const StateEstimation * const se, int16_t timestamp_ms, uint8_t sysid, float x, float y) {
#ifndef RPI_DBG
    mavlink_message_t msg;
    
    mavlink_msg_beacon_position_ned_pack(raw_data_logging_sysid,
			sysid,
			&msg,
			timestamp_ms,
			(float)x,
                        (float)y,
			0.0);
    mavlink_stream_send(se->mavlink_stream_, &msg);
#endif
}

void OfflineProcessing::sendRawBeaconPosition(const StateEstimation * const se, int16_t timestamp_ms, uint8_t sysid, float x, float y) {
#ifndef RPI_DBG
    mavlink_message_t msg;
    
    mavlink_msg_beacon_position_ned_pack(processed_data_logging_sysid,
			sysid,
			&msg,
			timestamp_ms,
			(float)x,
                        (float)y,
			0.0);
    mavlink_stream_send(se->mavlink_stream_, &msg);
#endif
}

void OfflineProcessing::sendProcessedGPSPosition(const StateEstimation * const se, int16_t timestamp_ms, Eigen::Vector3d position) {
#ifndef RPI_DBG
    float hdg_deg = h_getYawFromQuat(se->ekf_.state_.attitude_quat);
    local_coordinates_t local_pos;
    local_pos.pos.v[0] = position(0);
    local_pos.pos.v[1] = position(1);
    local_pos.pos.v[2] = position(2);
    
    local_pos.heading = hdg_deg;
    local_pos.origin = se->origin_;
    
    global_position_t gpos = coord_conventions_local_to_global_position(local_pos);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(processed_data_logging_sysid, 254,
							&msg,
							timestamp_ms,
							(int32_t)(((double)gpos.latitude) * 10000000),
							(int32_t)(((double)gpos.longitude) * 10000000),
							gpos.altitude * 1000.0f,
							-gpos.altitude * 1000,
							0, // vx 
							0, // vy
							0, // vz
							(uint16_t)hdg_deg // hdg
							);
    mavlink_stream_send(se->mavlink_stream_, &msg);
#endif
}

void OfflineProcessing::writeState(Vector3 position, Matrix3 cov, int64_t t_ms) {

    state_stream << (int)t_ms << ", "
            << position(0) << ", " << position(1) << ", "
            << cov(0,0) << ", " << cov(0,1) << ", " << cov(1,0) << ", " << cov(1,1) << "\n";
    state_stream.flush();
}

void OfflineProcessing::iros_2021_write_gps(Vector3 gps_pos, int64_t t_ms) {
  gps_stream << (int)t_ms << ", "
            << gps_pos(0) << ", " << gps_pos(1) << "\n";
  gps_stream.flush();
}

void OfflineProcessing::writeGPSUpdatePositions(Vector3 gps_pos, int64_t t_ms) {
  gps_updates_stream << (int)t_ms << ", "
            << gps_pos(0) << ", " << gps_pos(1) << "\n";
  gps_updates_stream.flush();
}


void OfflineProcessing::writeRangeMeasurement(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t t_ms) {
    aco_range_stream << (int)t_ms << ", " << (int)beacon_sys_id << ", " << beacon_pos(0) << ", " << beacon_pos(1) << ", " 
            << beacon_cov(0,0) << ", " << beacon_cov(0,1) << ", " << beacon_cov(1,0) << ", " << beacon_cov(1,1) << ", " 
            << range_m << "\n";
    aco_range_stream.flush();
}

