/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   offline-processing.h
 * Author: quraishi
 *
 * Created on 2 April, 2019, 4:02 PM
 */

#ifndef OFFLINE_PROCESSING_H
#define OFFLINE_PROCESSING_H

#include <stdint.h>
#include <fstream>
#include <ostream>
#include <sstream>
#include <iostream>

#include "Library/util/helper_utils.h"

class StateEstimation;

// Use a class to be able to friend the whole batch of functions
class OfflineProcessing {
public:
    void openFiles(std::string fn_prefix);
    
    void sendProcessedRangeMeasurements(const StateEstimation * const se, int16_t timestamp_ms, uint8_t sysid, float range_m);
    
    void sendProcessedBeaconPosition(const StateEstimation * const se, int16_t timestamp_ms, uint8_t sysid, float x, float y);
    void sendRawBeaconPosition(const StateEstimation * const se, int16_t timestamp_ms, uint8_t sysid, float x, float y);
    
    void sendProcessedGPSPosition(const StateEstimation * const se, int16_t timestamp_ms, Eigen::Vector3d position);
    
    void writeState(Vector3 position, Matrix3 cov, int64_t t_ms);
    void writeRangeMeasurement(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t t_ms);
    
    void iros_2021_write_gps(Vector3 gps_pos, int64_t t_ms);
    
    void writeGPSUpdatePositions(Vector3 gps_pos, int64_t t_ms);
    
    std::ofstream state_stream;
    std::ofstream gps_stream;
    std::ofstream gps_updates_stream;
    std::ofstream aco_range_stream;
    
};

#endif /* OFFLINE_PROCESSING_H */

