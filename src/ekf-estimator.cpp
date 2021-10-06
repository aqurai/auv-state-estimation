/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "ekf-estimator.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <chrono>

#include "plog/Log.h"
//#include <Eigen/Eigen>
//#include <Eigen/Dense>

//#include "graph_opt.h"
#include "debug_csv_write.h"
#include "Library/communication/mavlink_stream.h"
#include "Library/hal_emu/drivers/delay.h"
#include "Library/util/helper_utils.h"
#include "state_estimation.h"
#include "covariance_intersection.h"

EKFParams::EKFParams() {
    param_hdg_offset_deg = -5.0f; // -55
    param_thrust_K = 8.10f;
    param_mass = 7.0f;
    param_drag_coef_X = 1.71f;
    param_drag_coef_Y = .155f;
    param_drag_coef_Z = 0.0f;
    param_stream_vel_x = 0.0f;
    param_stream_vel_y = 0.0f;
}

EKFEstimator::EKFEstimator() {
    reset(Vector3::Zero(), 0);
}

EKFEstimator::EKFEstimator(const EKFEstimator& other) {
    reset(Vector3::Zero(), 0);
    
    state_.state_ = other.state_.state_;
    state_.covariance_ = other.state_.covariance_;
   

    state_.attitude_quat = other.state_.attitude_quat;
    state_.attitude_covariance_ = other.state_.attitude_covariance_;
    state_.thrust_left = other.state_.thrust_left;
    state_.thrust_right = other.state_.thrust_right;

    state_.timestamp_ms_ = other.state_.timestamp_ms_;
}

EKFEstimator& EKFEstimator::operator= (const EKFEstimator& other) {
    state_.state_ = other.state_.state_;
    state_.covariance_ = other.state_.covariance_;
   

    state_.attitude_quat = other.state_.attitude_quat;
    state_.attitude_covariance_ = other.state_.attitude_covariance_;
    state_.thrust_left = other.state_.thrust_left;
    state_.thrust_right = other.state_.thrust_right;

    state_.timestamp_ms_ = other.state_.timestamp_ms_;
}

void EKFEstimator::reset(Vector3 start_pos, int64_t timestamp_ms) {
    state_.position_ = start_pos;
    state_.timestamp_ms_ = timestamp_ms;
    state_.velocity_ = Vector3::Zero();
    state_.acceleration_ = Vector3::Zero();
    state_.covariance_ = Matrix9::Identity() * 0.001f;
    
    state_.attitude_covariance_ = Matrix3::Identity() * 0.001f;
}

void EKFEstimator::setAttitude(QuatType q_attitude, const EKFParams& p) {
    QuatType heading_corr ( AngleAxisType(p.param_hdg_offset_deg*M_PI/180.0f, Vector3::UnitZ()));
    state_.attitude_quat = heading_corr * q_attitude;
}


void EKFEstimator::setMotorSpeed(SEFloatingType left_motor, SEFloatingType right_motor, int64_t timestamp_ms, const EKFParams& p) {
    SEFloatingType left_angvel = left_motor;
    SEFloatingType right_angvel = right_motor;
    state_.thrust_left = p.param_thrust_K * h_sign(left_angvel) * left_angvel * left_angvel;
    state_.thrust_right = p.param_thrust_K * h_sign(right_angvel) * right_angvel * right_angvel;
}


void EKFEstimator::predictStep(int64_t timestamp_ms, const EKFParams& p) {
    SEFloatingType thrust_L = state_.thrust_left;
    SEFloatingType thrust_R = state_.thrust_right;
    
    SEFloatingType dt_s = (SEFloatingType)(timestamp_ms - state_.timestamp_ms_) / 1000.0f;
    if (dt_s <= 0) {
        return;
    }
    state_.timestamp_ms_ = timestamp_ms;
    
    Vector3 stream_gf = Vector3(p.param_stream_vel_x, p.param_stream_vel_y, 0.0f);
    Vector3 stream_bf = globalToLocal(state_.attitude_quat, stream_gf);
    
    Vector3 v_bf = globalToLocal(state_.attitude_quat, state_.velocity_) - stream_bf;
    
    if (v_bf.norm() > 2) {
        v_bf = 2*v_bf / v_bf.norm();
    }
        
    state_.acceleration_(0) = (thrust_L + thrust_R) - (h_sign(v_bf(0)) * v_bf(0) * v_bf(0) * p.param_drag_coef_X ); // / p.param_mass;
    state_.acceleration_(1) = (- h_sign(v_bf(1)) * v_bf(1) * v_bf(1) * p.param_drag_coef_Y); // / p.param_mass;
    state_.acceleration_(2) = (- h_sign(v_bf(2)) * v_bf(2) * v_bf(2) * p.param_drag_coef_Z);  // / p.param_mass;
    
    
    
    state_.velocity_ = state_.velocity_ * 0.1f;
    
    state_.velocity_ = state_.velocity_ + dt_s * localToGlobal(state_.attitude_quat, state_.acceleration_);
    
   
    state_.position_ = state_.position_ + dt_s * state_.velocity_;
    
//    std::cout << "State accelera: " << state_.acceleration_.transpose() << " .. Motor LR = " << state_.thrust_left << ", " << state_.thrust_right << "\n";
//    std::cout << "State velocity: " << state_.velocity_.transpose() << "\n";
//    std::cout << "State position: " << state_.position_.transpose() << "\n";
//    
    // Covariance calculation
    Matrix9 pred_jacobian_;
    Matrix9 motion_noise = Matrix9::Zero();
    Matrix3 R = state_.attitude_quat.toRotationMatrix();
    
    SEFloatingType d_kx = -2 * h_sign(v_bf(0)) * v_bf(0) * p.param_drag_coef_X / p.param_mass;
    SEFloatingType d_ky = -2 * h_sign(v_bf(1)) * v_bf(1) * p.param_drag_coef_Y / p.param_mass;
    SEFloatingType d_kz = -2 * h_sign(v_bf(2)) * v_bf(2) * p.param_drag_coef_Z / p.param_mass;
    
    Matrix3 del_abf_by_del_vgf = (Vector3(d_kx, d_ky, d_kz)).asDiagonal() * R.transpose();
    
    pred_jacobian_ << Matrix3::Identity(),      dt_s * Matrix3::Identity(),     Matrix3::Zero(),
                      Matrix3::Zero(),          Matrix3::Identity(),            dt_s * R,
                      Matrix3::Zero(),          0.0*del_abf_by_del_vgf,             Matrix3::Zero();
    // Webots has this accleration measurement problem, where a=0 most of the time and a= ~300 for short durations. This is 
    // Probably causing problems with this abf-vgf jacobian term.
    
    Vector3 motion_noise_vec(p.param_acc_prediciton_per_sec[0], p.param_acc_prediciton_per_sec[1], p.param_acc_prediciton_per_sec[2]);
    motion_noise.block<3,3>(6,6) = motion_noise_vec.asDiagonal();
    
    // Propagate attitude covariance
    // Assume error only in yaw
    SEFloatingType yaw_var = p.param_attitude_variance[2];
    Vector3 acc_err_calc = state_.acceleration_;
    acc_err_calc(2) = 0;
    
    // Project uncertainty due to attitude on acc measurement.
    Vector3 J_attitude;
    J_attitude << -acc_err_calc(1), acc_err_calc(0), 0;
    motion_noise.block<3,3>(6,6) = motion_noise.block<3,3>(6,6) + dt_s * J_attitude * yaw_var * J_attitude.transpose();
    
    state_.covariance_ = pred_jacobian_ * state_.covariance_ * pred_jacobian_.transpose() + motion_noise;
}

SEFloatingType EKFEstimator::imuUpdate(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms, const EKFParams& p) {
    predictStep(timestamp_ms, p);
    
    // H = [0_3x3  0_3x3, I_3x3] (3x9 matrix)
    Vector3 acc_variance(p.param_accelero_variance[0], p.param_accelero_variance[1], p.param_accelero_variance[2]);

    Matrix3 to_invert =  acc_variance.asDiagonal();
    
    to_invert = to_invert + state_.covariance_.block<3,3>(6,6);
    
    Matrix9_3 K_gain = state_.covariance_.block<9,3>(0,6) * to_invert.inverse();
    
    Vector3 up_gf(0,0,-1);
    Vector3 up_bf = globalToLocal(state_.attitude_quat, up_gf);

    Vector3 linear_acc = accelero - 9.81 * up_bf;
    
    if (accelero(0) < -0.5f) {
        return 0;
    }
    Vector3 e_acc = linear_acc - state_.acceleration_;
    
    state_.state_ = state_.state_ + K_gain * e_acc;
    
    Matrix9 Kgain_x_H = Matrix9::Zero();
    Kgain_x_H.block<9,3>(0,6) = K_gain;
    
    state_.covariance_ = state_.covariance_ - Kgain_x_H*state_.covariance_;
    
    return e_acc.norm();
}

SEFloatingType EKFEstimator::depthUpdate(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms, const EKFParams& p) {
    predictStep(timestamp_ms, p);
    
    // H = [0 0 1 0 0 0,
    //      0 0 0 0 0 1];
    
    // TODO(Anwar): Calculate depth vel from filtered/averaged depth measurements.
    // Setting to zero here.
    depth_vel = 0.0f;
    
    Matrix2 m_cov = Matrix2::Zero();
    m_cov << state_.covariance_(2, 2), state_.covariance_(2, 5),
            state_.covariance_(5, 2), state_.covariance_(5, 5);
    m_cov(0,0) = m_cov(0,0) + p.param_depth_variance[0];
    m_cov(1,1) = m_cov(1,1) + p.param_depth_variance[1];

    Matrix9_2 pre_m = Matrix9_2::Zero();
    pre_m.col(0) = state_.covariance_.col(2);
    pre_m.col(1) = state_.covariance_.col(5);
    
    Matrix9_2 k_gain = pre_m * m_cov.inverse();
    
    Vector2 error;
    error(0) = depth_m - state_.position_(2);
    error(1) = depth_vel - state_.velocity_(2);

    state_.state_ = state_.state_ + k_gain * error;
    
    Matrix9 KH = Matrix9::Zero();
    KH.col(2) = k_gain.col(0);
    KH.col(5) = k_gain.col(1);
    
    state_.covariance_ = state_.covariance_ - KH*state_.covariance_;
    
    return error.norm();
    
}

SEFloatingType EKFEstimator::gpsUpdate(Vector2 pos, int64_t timestamp_ms, const EKFParams& p) {
    predictStep(timestamp_ms, p);
    
    // H = [1 0 0 0 0 0, ...
    //      0 1 0 0 0 0 ...];
    Matrix2 m_cov = Matrix2::Zero();
    m_cov = state_.covariance_.block<2,2>(0,0);
    m_cov(0,0) = m_cov(0,0) + p.param_gps_variance[0];
    m_cov(1,1) = m_cov(1,1) + p.param_gps_variance[1];
    
    Matrix9_2 pre_m = Matrix9_2::Zero();
    pre_m = state_.covariance_.block<9,2>(0,0);
    
    Matrix9_2 k_gain = pre_m * m_cov.inverse();
        
    Vector2 error;
    error(0) = pos(0) - state_.position_(0);
    error(1) = pos(1) - state_.position_(1);
    
    state_.state_ = state_.state_ + k_gain * error;
    
    Matrix9 KH = Matrix9::Zero();
    KH.col(0) = k_gain.col(0);
    KH.col(1) = k_gain.col(1);
    
    state_.covariance_ = state_.covariance_ - KH*state_.covariance_;
    //std::cout << "GPS update, hdg err = " << p.param_hdg_offset_deg << ".  GPS pos = " << pos.transpose() << ",  est position = " << state_.position_.transpose() << ",  err = " << error.norm() << "\n";
    return error.norm();
}


SEFloatingType EKFEstimator::rangeUpdate(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms, const EKFParams& p) {
    predictStep(timestamp_ms, p);
    
    Vector9 posr;
    Matrix9 covar;
    
    std::tie(posr, covar) = rangeBasedCovarianceIntersection(state_.state_, state_.covariance_, beacon_pos, beacon_cov, range_m);
    
    Vector3 err = (posr - state_.state_).block<3,1>(0,0);
    state_.state_ = posr;
    state_.covariance_ = covar;
    return err.norm();
}

SEFloatingType EKFEstimator::rangeBearingUpdate(Vector3 local_pos_2d, Matrix2 cov, int64_t timestamp_ms, const EKFParams& p) {
    predictStep(timestamp_ms, p);
    
    // H = [1 0 0 0 0 0, ...
    //      0 1 0 0 0 0 ...];
    Matrix2 m_cov = Matrix2::Zero();
    m_cov = state_.covariance_.block<2,2>(0,0);
    m_cov = m_cov + cov;
    
    Matrix9_2 pre_m = Matrix9_2::Zero();
    pre_m = state_.covariance_.block<9,2>(0,0);
    
    Matrix9_2 k_gain = pre_m * m_cov.inverse();
        
    Vector2 error;
    error(0) = local_pos_2d(0) - state_.position_(0);
    error(1) = local_pos_2d(1) - state_.position_(1);
    
    state_.state_ = state_.state_ + k_gain * error;
    
    Matrix9 KH = Matrix9::Zero();
    KH.col(0) = k_gain.col(0);
    KH.col(1) = k_gain.col(1);
    
    state_.covariance_ = state_.covariance_ - KH*state_.covariance_;
    //std::cout << "GPS update, hdg err = " << p.param_hdg_offset_deg << ".  GPS pos = " << pos.transpose() << ",  est position = " << state_.position_.transpose() << ",  err = " << error.norm() << "\n";
    return error.norm();
}

Vector3 EKFEstimator::getPosition() const {
    return state_.position_;
}

