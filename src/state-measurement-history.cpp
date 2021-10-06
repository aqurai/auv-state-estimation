/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "state-measurement-history.h"
#include "debug_csv_write.h"

#include <Eigen/Dense>
#include "plog/Log.h"
#include "state_estimation.h"

#include <string>
#include <vector>
#include <iterator>
#include <cmath>
#include <math.h>
#include <plog/Log.h>



MotorCommandData::MotorCommandData(SEFloatingType left, SEFloatingType right, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::MotorCommand) {
    timestamp_ms_ = timestamp_ms;
    motor_left_ = left;
    motor_right_ = right;
}

IMUData::IMUData(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::IMU) {
    timestamp_ms_ = timestamp_ms;
    accelerometer_ = accelero;
    gyroscope_ = gyro;
}

AttitudeData::AttitudeData(QuatType att_quat, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::Attitude) {
    timestamp_ms_ = timestamp_ms;
    attitude_quat_ = att_quat;
}

GPSLocalData::GPSLocalData(Vector3 gps_local_pos, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::GPSLocal) {
    timestamp_ms_ = timestamp_ms;
    gps_local_pos_ = gps_local_pos;
}

DepthSensorData::DepthSensorData(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::DepthSensor) {
    timestamp_ms_ = timestamp_ms;
    depth_m_ = depth_m;
    depth_vel_ = depth_vel;
}

AcousticRangeData::AcousticRangeData(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::AcousticRange) {
    timestamp_ms_ = timestamp_ms;
    beacon_sys_id_ = beacon_sys_id;
    beacon_pos_ = beacon_pos;
    beacon_cov_ = beacon_cov;
    range_m_ = range_m;
}

AcousticRangeBearingData::AcousticRangeBearingData(Vector3 local_pos_2d, Matrix2 cov, int64_t timestamp_ms) :
    StateTransitionData(UpdateType::AcousticRangeBearing) {
    timestamp_ms_ = timestamp_ms;
    local_pos_2d_ = local_pos_2d;
    cov_ = cov;
}

SEFloatingType MotorCommandData::updateState(EKFEstimator *state, const EKFParams& params) {
    state->setMotorSpeed(motor_left_, motor_right_, timestamp_ms_, params);
//    std::cout  << "Motor update at " << (int)timestamp_ms_ << " ms. Params: \n";
//    std::cout  << "                motor left :" << motor_left_ << "\n";
//    std::cout  << "                motor right:" << motor_right_ << "\n";
    
    return 0.0f;
}

SEFloatingType IMUData::updateState(EKFEstimator *state, const EKFParams& params) {
//    std::cout  << "IMU update at " << (int)timestamp_ms_ << " ms. Params: \n";
//    std::cout  << "                acc :" << accelerometer_.norm() << "\n";
//    std::cout  << "                gyro:" << gyroscope_.norm() << "\n";
    
    return state->imuUpdate(accelerometer_, gyroscope_, timestamp_ms_, params);
}

SEFloatingType AttitudeData::updateState(EKFEstimator *state, const EKFParams& params) {
//    std::cout  << "Att update at " << (int)timestamp_ms_ << " ms. Params: \n";
    
    state->setAttitude(attitude_quat_, params);
    return 0.0f;
}

SEFloatingType GPSLocalData::updateState(EKFEstimator *state, const EKFParams& params) {
//    std::cout  << "GPS update at " << (int)timestamp_ms_ << " ms. Params: \n";
    
    return state->gpsUpdate(Vector2(gps_local_pos_(0), gps_local_pos_(1)), timestamp_ms_, params);
}

SEFloatingType DepthSensorData::updateState(EKFEstimator *state, const EKFParams& params) {
//    std::cout  << "Depth update at " << (int)timestamp_ms_ << " ms. Params: \n";
//    std::cout  << "                depth :" << depth_m_ << "\n";
    
    return state->depthUpdate(depth_m_, depth_vel_, timestamp_ms_, params);
}

SEFloatingType AcousticRangeData::updateState(EKFEstimator *state, const EKFParams& params) {
//    std::cout  << "Acoustic update at " << (int)timestamp_ms_ << " ms. Params: \n";
//    std::cout  << "                beacon id  :" << (int)beacon_sys_id_ << "\n";
//    std::cout  << "                beacon pos :" << beacon_pos_(0) << ", " << beacon_pos_(1) << "\n";
//    std::cout  << "                range      :" << range_m_ << "\n";
    
    SEFloatingType retval =  state->rangeUpdate(beacon_sys_id_, beacon_pos_, beacon_cov_, range_m_, timestamp_ms_, params);
    return retval;
}

SEFloatingType AcousticRangeBearingData::updateState(EKFEstimator *state, const EKFParams& params) {
    
    SEFloatingType retval =  state->rangeBearingUpdate(local_pos_2d_, cov_, timestamp_ms_, params);
    return retval;
}


StateHistoryManager::StateHistoryManager() {

}

void StateHistoryManager::reset() {
    state_frames_.clear();
}

bool compare_predicate(int64_t t_ms, StateTransitionData* d_2) {
    return t_ms < d_2->timestamp_ms_;
};
    
void StateHistoryManager::insertDataElement(StateTransitionData* d) {
    //std::cout  << "This needs to be verified.";
    auto upper_it = std::upper_bound(transition_data_history_.begin(), transition_data_history_.end(), d->timestamp_ms_, compare_predicate);
    transition_data_history_.insert(upper_it, d);
}

void StateHistoryManager::insertStateFrame(EKFEstimator *ekf_state, int64_t timestamp_ms) {
    state_frames_[timestamp_ms] = *ekf_state;
}

int64_t StateHistoryManager::getLastStateFrameTimestampMs() {
    if (state_frames_.empty()) {
        return -1;
    } else {
        return state_frames_.rbegin()->first;
    }
}


SEFloatingType StateHistoryManager::evaluateSingleUpdateError(const EKFEstimator& state, StateTransitionData* transition, int parameter_set) {
    LOG_WARNING << "Not implemented";
}

SEFloatingType StateHistoryManager::evaluateChainUpdateError(int64_t start_ms, const EKFParams& params, const std::set<UpdateType>& update_types) {
    LOG_WARNING << "Not implemented";
}

SEFloatingType StateHistoryManager::evaluateChainUpdateError(int64_t start_ms, const EKFParams& params) {
    auto state_it = state_frames_.lower_bound(start_ms);
    
    if (state_it == state_frames_.end()) {
        return 0.0f;
    }
    if (state_it != state_frames_.begin()) {
        --state_it;
    }
    
    EKFEstimator ekfs(state_it->second);
    
    auto data_it = std::upper_bound(transition_data_history_.begin(), transition_data_history_.end(), state_it->first, compare_predicate);
    
    SEFloatingType total_err = 0.0f;
    for (auto it=data_it; it!=transition_data_history_.end(); ++it) {
        
        SEFloatingType e = (*it)->updateState(&ekfs, params);
        
        if ((*it)->getUpdateType() == UpdateType::AcousticRange  ||
                (*it)->getUpdateType() == UpdateType::GPSLocal || 
                (*it)->getUpdateType() == UpdateType::AcousticRangeBearing) {
            total_err = total_err + e;
        }
    }
    return total_err;
}

SEFloatingType StateHistoryManager::stateUpdate(EKFEstimator* state, StateTransitionData* transition) {
    return 0; //transition->updateState(state);
}



