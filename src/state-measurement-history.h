/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   graph_opt.h
 * Author: quraishi
 *
 * Created on 31 August, 2018, 2:20 PM
 */

#ifndef GRAPH_OPT_H
#define GRAPH_OPT_H

#include <vector>
#include <map>
#include <set>
#include <iterator>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "Library/util/helper_utils.h"
#include "ekf-estimator.h"

enum class UpdateType {
    Default,
    MotorCommand,
    IMU,
    Attitude,
    GPSLocal,
    DepthSensor,
    AcousticRange,
    AcousticRangeBearing,
};

class StateTransitionData {
public:
    StateTransitionData(UpdateType type) : update_type(type) {}
    virtual UpdateType getUpdateType() const { return update_type; }
    virtual SEFloatingType updateState(EKFEstimator *state, const EKFParams& p) = 0;
    int64_t timestamp_ms_;
private:
    const UpdateType update_type;
};

class MotorCommandData : public StateTransitionData {
public:
    MotorCommandData(SEFloatingType left, SEFloatingType right, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    SEFloatingType motor_left_;
    SEFloatingType motor_right_;
};

class IMUData : public StateTransitionData {
public:        
    IMUData(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    Vector3 accelerometer_;
    Vector3 gyroscope_;
};

class AttitudeData : public StateTransitionData {
public:        
    AttitudeData(QuatType att_quat, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    QuatType attitude_quat_;
};

class GPSLocalData : public StateTransitionData {
public:        
    GPSLocalData(Vector3 gps_local_pos, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    Vector3 gps_local_pos_;
};

class DepthSensorData : public StateTransitionData {
public:        
    DepthSensorData(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    SEFloatingType depth_m_;
    SEFloatingType depth_vel_;
};

class AcousticRangeData : public StateTransitionData {
public:        
    AcousticRangeData(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    uint8_t beacon_sys_id_;
    Vector3 beacon_pos_;
    Matrix2 beacon_cov_;
    SEFloatingType range_m_;
};

class AcousticRangeBearingData : public StateTransitionData {
public:        
    AcousticRangeBearingData(Vector3 local_pos_2d, Matrix2 cov, int64_t timestamp_ms);
    SEFloatingType updateState(EKFEstimator *state, const EKFParams& params);
    Vector3 local_pos_2d_;
    Matrix2 cov_;
};


class StateHistoryManager {
public:
    StateHistoryManager();;
    void reset();
    void insertDataElement(StateTransitionData* d); // then mark subsequent states as not computed. or better still, recmopute/delete them.
    void insertStateFrame(EKFEstimator *ekf_state, int64_t timestamp_ms);
    
    int64_t getLastStateFrameTimestampMs();
    
    // Error for a particular measurement/update, at a particular time instant
    SEFloatingType evaluateSingleUpdateError(const EKFEstimator& state, StateTransitionData* transition, int parameter_set);
    
    // between the {start, end} time, for a given set of parameters, for listed kinds of updates
    SEFloatingType evaluateChainUpdateError(int64_t start_ms, const EKFParams& params, const std::set<UpdateType>& update_types);
    SEFloatingType evaluateChainUpdateError(int64_t start_ms, const EKFParams& params);
    
    std::vector<StateTransitionData*> transition_data_history_;
    
private:
    SEFloatingType stateUpdate(EKFEstimator *state, StateTransitionData* transition);
    
    std::map<int64_t, EKFEstimator> state_frames_;
};


#endif /* GRAPH_OPT_H */

