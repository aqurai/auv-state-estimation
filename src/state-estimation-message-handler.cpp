/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "state-estimation-message-handler.h"
#include "Library/communication/mavlink_stream.h"
#include "Library/communication/mavlink_message_handler.h"
#include "state_estimation.h"
#include "debug_csv_write.h"
#include "Library/communication/mavlink_stream.h"
#include "Library/util/helper_utils.h"

#include "plog/Log.h"


void state_estimation_receive_range_measurement(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_beacon_position(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_integrated_position(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_origin(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_raw_imu(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_servo_output_raw(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_beacon_position_ned_radio(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_aco_decoded_data(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_mav_heartbeat(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_dev_receive_real_position(StateEstimation * state_estimation, mavlink_received_t* rec);
void state_estimation_receive_heading_from_gps_pos(StateEstimation * state_estimation, mavlink_received_t* rec);



void state_estimation_receive_range_measurement(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    mavlink_beacon_range_measurement_t packet;
    mavlink_msg_beacon_range_measurement_decode(&rec->msg, &packet);
//    state_estimation->receiveRangeMeasurement(packet.system, (double)packet.range_m, (int64_t)packet.time_boot_ms);
}

void state_estimation_receive_beacon_position(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    mavlink_beacon_position_gps_t packet;
    mavlink_msg_beacon_position_gps_decode(&rec->msg, &packet);
    global_position_t beacon_pos;
    beacon_pos.latitude = (double)packet.lat * 0.0000001;
    beacon_pos.longitude = (double)packet.lon * 0.0000001;
    beacon_pos.altitude = (double)packet.alt * 0.001f;
    beacon_pos.heading = 0.0f;
    //state_estimation->updateBeacon(rec->msg.sysid, beacon_pos, (int64_t)packet.time_boot_ms);
    
    Vector3 pos_ned = h_gpsToVect(beacon_pos, state_estimation->origin_);
    
    write_beacon_position_ned((int64_t)packet.time_boot_ms, pos_ned(0), pos_ned(1), rec->msg.sysid);
}

void state_estimation_receive_origin(StateEstimation * state_estimation, mavlink_received_t* rec) {
//    if (!state_estimation) return;
//    mavlink_gps_global_origin_t packet;
//    mavlink_msg_gps_global_origin_decode(&rec->msg, &packet);
//    global_position_t origin;
//    origin.latitude = (double)packet.latitude * 0.0000001;
//    origin.longitude = (double)packet.longitude * 0.0000001;
//    origin.altitude = (double)packet.altitude * 0.001f;
//    origin.heading = 0.0f;
//    state_estimation->updateOrigin(origin);
}

void state_estimation_receive_heading_from_gps_pos(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&rec->msg, &packet);
    double heading = (double)packet.hdg;
    //state_estimation->receiveHeading(heading, packet.time_boot_ms);
}

void state_estimation_receive_mav_heartbeat(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    if (rec->msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
            return;
    }
    mavlink_heartbeat_t packet;	
    mavlink_msg_heartbeat_decode(&rec->msg, &packet);
    
    bool armed = packet.base_mode >> 7;
    LOG_DEBUG << "Received mode byte " << (int)packet.base_mode << ", bit 7: " << armed << ", from system " << (int)rec->msg.sysid << ":" << (int)rec->msg.compid;
    state_estimation->receiveModeByte(packet.base_mode);
}

void state_estimation_receive_raw_imu(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    if (rec->msg.msgid != MAVLINK_MSG_ID_RAW_IMU) {
            return;
    }
    if (rec->msg.sysid == state_estimation->mavlink_stream_->sysid) {
        return;
    }
    mavlink_raw_imu_t packet;
    mavlink_msg_raw_imu_decode(&rec->msg, &packet);
    
}

void state_estimation_receive_servo_output_raw(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    if (rec->msg.msgid != MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
            return;
    }
    if (rec->msg.sysid == state_estimation->mavlink_stream_->sysid) {
        return;
    }
    mavlink_servo_output_raw_t packet;
    mavlink_msg_servo_output_raw_decode(&rec->msg, &packet);
    
    
}

void state_estimation_receive_beacon_position_ned_radio(StateEstimation * state_estimation, mavlink_received_t* rec) {
    /*if (!state_estimation) return;
    if (rec->msg.msgid != MAVLINK_MSG_ID_BEACON_POSITION_NED) {
            return;
    }
    if (rec->msg.sysid == state_estimation->mavlink_stream_->sysid) {
        return;
    }
    mavlink_beacon_position_ned_t packet;
    mavlink_msg_beacon_position_ned_decode(&rec->msg, &packet);
    
    state_estimation->receiveBeaconNavData((uint8_t)rec->msg.sysid, (double)packet.x, (double)packet.y, (int64_t)packet.time_boot_ms);*/
  
    // Moved to beacon manager
}

void state_estimation_receive_aco_decoded_data(StateEstimation * state_estimation, mavlink_received_t* rec) {
    /*if (!state_estimation) return;
    if (rec->msg.msgid != MAVLINK_MSG_ID_RAW_DATA_STREAM_8) {
            return;
    }
    mavlink_raw_data_stream_8_t packet;
    mavlink_msg_raw_data_stream_8_decode(&rec->msg, &packet);
    
    int16_t bitscores[36];
    for (int k=0; k<36; ++k){
       bitscores[k] = packet.values[k]; 
    }
    uint8_t sender_sysid = packet.values[34];
    
    int16_t x_aco = (int16_t)packet.values[32] - 127;
    int16_t y_aco = (int16_t)packet.values[33] - 127;

    LOG_WARNING << "==> Received aco pos from sys "  << (int)sender_sysid << " : " << x_aco << ", " << y_aco << "   Time: " << packet.time_boot_ms;
    
    state_estimation->updateBeaconPositionNEDAcoustics(sender_sysid, (double)x_aco, (double)y_aco, (int64_t)packet.time_boot_ms, bitscores);*/
    
    // Moved to beacon manager
}


void state_estimation_dev_receive_real_position(StateEstimation * state_estimation, mavlink_received_t* rec) {
    if (!state_estimation) return;
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&rec->msg, &packet);
    global_position_t gps_pos;
    gps_pos.latitude = (double)packet.lat * 0.0000001;
    gps_pos.longitude = (double)packet.lon * 0.0000001;
    gps_pos.altitude = (double)packet.alt * 0.001f;
    gps_pos.heading = 0.0f;
    
    Vector3 pos_ned = h_gpsToVect(gps_pos, state_estimation->origin_);
    state_estimation->real_positions_.push_back(pos_ned);
    
    write_real_position_ned((int64_t)packet.time_boot_ms, pos_ned(0), pos_ned(1));
    
    
    Vector3 real_robot_pos = pos_ned;
    
    double n = 0.0;
    for(auto pos : state_estimation->real_positions_) {
        real_robot_pos += pos;
        n = n + 1.0;
        real_robot_pos = real_robot_pos/n;
    }
    
    real_robot_pos = 0.3*real_robot_pos + 0.7 * pos_ned;
    
    Vector3 beacon_pos(20, 45.0, 0);
    //state_estimation->test_addSimulatedBeaconMeasurement(real_robot_pos, beacon_pos, 10);
    
    if (state_estimation->real_positions_.size() > 5) {
        state_estimation->real_positions_.erase(state_estimation->real_positions_.begin());
    }
    
}

void setupStateEstimatorCallabcks(StateEstimation *se, mavlink_message_handler_t *message_handler, uint8_t self_sysid) {
    
    mavlink_register_callback(self_sysid, MAV_COMP_ID_ALL, 
            MAVLINK_MSG_ID_BEACON_RANGE_MEASUREMENT,
            state_estimation_receive_range_measurement, se);
    
    mavlink_register_callback(MAV_SYS_ID_ALL, MAV_COMP_ID_ACOUSTIC, 
            MAVLINK_MSG_ID_BEACON_POSITION_GPS,
            state_estimation_receive_beacon_position, se);
    
    mavlink_register_callback(self_sysid, MAV_COMP_ID_MAIN, 
            MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
            state_estimation_receive_origin, se);
    
    mavlink_register_callback(self_sysid, MAV_COMP_ID_MAIN, 
            MAVLINK_MSG_ID_RAW_IMU,
            state_estimation_receive_raw_imu, se);

    mavlink_register_callback(self_sysid, MAV_COMP_ID_MAIN, 
            MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            state_estimation_receive_servo_output_raw, se);
    
    mavlink_register_callback(MAV_SYS_ID_ALL, MAV_COMP_ID_ACOUSTIC, 
            MAVLINK_MSG_ID_BEACON_POSITION_NED,
            state_estimation_receive_beacon_position_ned_radio, se);
    
    mavlink_register_callback(self_sysid, MAV_COMP_ID_ACOUSTIC, 
            MAVLINK_MSG_ID_RAW_DATA_STREAM_8,
            state_estimation_receive_aco_decoded_data, se);

    
#ifdef REAL_SYS
    mavlink_register_callback(self_sysid, MAV_COMP_ID_MAIN, 
            MAVLINK_MSG_ID_HEARTBEAT, // This is zero
            state_estimation_receive_mav_heartbeat, se);
    
#endif
    
#ifdef PC_EMULATION
    
    mavlink_register_callback(self_sysid, 101, 
            MAVLINK_MSG_ID_GLOBAL_POSITION_INT
            state_estimation_dev_receive_real_position, se);
    
    mavlink_register_callback(self_sysid, 101, 
            MAVLINK_MSG_ID_GLOBAL_POSITION_INT
            state_estimation_receive_heading_from_gps_pos, se);
#endif
    
}
