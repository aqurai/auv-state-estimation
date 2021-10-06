/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ekf.h
 * Author: quraishi
 *
 * Created on 10 March, 2019, 4:06 PM
 */

#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <stdint.h>
#include <vector>

#include <Eigen/Eigen>
#include "Library/util/helper_utils.h"

class EKFParams {
public:
    EKFParams();
    
    SEFloatingType param_hdg_offset_deg;
    SEFloatingType param_thrust_K;
    SEFloatingType param_mass;
    SEFloatingType param_drag_coef_X;
    SEFloatingType param_drag_coef_Y;
    SEFloatingType param_drag_coef_Z;
    SEFloatingType param_acc_prediciton_per_sec[3] = {0.05f, 0.05f, 0.05f};
    SEFloatingType param_accelero_variance[3] = {1.0f, 1.0f, 1.0f};
    SEFloatingType param_attitude_variance[3] = {0.01f, 0.01f, 0.1f};
    SEFloatingType param_depth_variance[2] = {0.1f, 3.0f};
    SEFloatingType param_gps_variance[2] = {1.0f, 1.0f};
    SEFloatingType param_stream_vel_x;
    SEFloatingType param_stream_vel_y;
};

class EKFEstimator {
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EKFEstimator();
    EKFEstimator(const EKFEstimator& other);
    EKFEstimator& operator= (const EKFEstimator& other);
    
    void reset(Vector3 start_pos, int64_t timestamp_ms);
    
    void setAttitude(QuatType q_attitude, const EKFParams& p);
    
    void setMotorSpeed(SEFloatingType left_motor, SEFloatingType right_motor, int64_t timestamp_ms, const EKFParams& p);
    
    void predictStep(int64_t timestamp_ms, const EKFParams& p);
    
    SEFloatingType imuUpdate(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms, const EKFParams& p);
    SEFloatingType depthUpdate(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms, const EKFParams& p);
    SEFloatingType gpsUpdate(Vector2 pos, int64_t timestamp_ms, const EKFParams& p);
    SEFloatingType rangeUpdate(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms, const EKFParams& p);
    SEFloatingType rangeBearingUpdate(Vector3 local_pos_2d, Matrix2 cov, int64_t timestamp_ms, const EKFParams& p);
    
    Vector3 getPosition() const;
    
public:

    struct EKFState {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EKFState() :
            position_(&state_(0)), 
            velocity_(&state_(3)),
            acceleration_(&state_(6)) {}
        
        EKFState& operator= (const EKFState& other) = delete;
        
        // State variables. 9x1 state
        Vector9 state_;
        Matrix9 covariance_;

        // Aux variables
        Eigen::Map<Vector3> position_;
        Eigen::Map<Vector3> velocity_;
        Eigen::Map<Vector3> acceleration_;
        
        QuatType attitude_quat;
        Matrix3 attitude_covariance_;
        SEFloatingType thrust_left;
        SEFloatingType thrust_right;
        
        int64_t timestamp_ms_;
      
    };
    EKFState state_;
    
    
    
    
};

#endif /* EKF_ESTIMATOR_H */

