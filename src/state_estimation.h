/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   state_estimation.h
 * Author: quraishi
 *
 * Created on 29 August, 2018, 1:24 PM
 */

#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <map>
#include <queue>
#include <stdint.h>
#include <string>

#include <Eigen/Eigen>

#include "Library/communication/mavlink_stream.h"
#include "Library/util/coord_conventions.h"
#include "Library/util/helper_utils.h"
#include "state-measurement-history.h"
#include "beacon_manager.h"
#include "ekf-estimator.h"
#include "offline-processing.h"
#include "imu-calibration.h"

class StateEstimation {
  friend class OfflineProcessing;
public:
  StateEstimation(global_position_t origin, uint8_t self_sysid, mavlink_stream_t* mavlink_stream, std::string fn_prefix);

  void resetPosition(Vector3 pos);

  void sendPositionGps(Vector3 position, uint32_t timestamp_ms);

  void sendPositionGps();

  void receiveIMU(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms);
  void receiveAttitude(QuatType attitide_quat, int64_t timestamp_ms);
  void receiveMotorCommands(SEFloatingType motor_left, SEFloatingType motor_right, int64_t timestamp_ms);
  void receiveGPSLocalPosition(Vector3 gpsPosLocal, int64_t timestamp_ms);
  void receiveRangeMeasurement(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms);
  void receiveDepth(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms);
  void receiveRangeBearingMeasurement(Vector3 pos_2d, Matrix2 cov, int64_t timestamp_ms);
  void receiveBeaconNavData(uint8_t beacon_sysid, Vector3 position, Matrix2 covariance, SEFloatingType heading_deg, SEFloatingType range_m, uint8_t to_sysid, time_milliseconds_t t_ms);

  uint8_t getBestBeaconId();


  // Range measurements coming soon.

  // Additional Mavlink callbacks for real systems
  // ---------------------------------------------
  void updateOrigin(global_position_t& global_origin);
  void receiveModeByte(uint8_t base_mode);
  void receiveIMUPreintegrated(Vector3 position, Eigen::Matrix3f covariance, int64_t timestamp_ms);
  void updateBeaconPositionNEDAcoustics(uint8_t sys_id, double x, double y, int64_t timestamp_ms, int16_t *bitscores);

  Vector3 getPosition();
  Matrix3 getPositionCovariance();
  SEFloatingType getYaw();
  QuatType getAttitude();

  bool isArmed() {
    return armed_;
  }

  // PC / post processing functions
  void initializeStartingPosition(Vector3 gpsPosLocal, int64_t timestamp_ms);
  int64_t gps_init_time_ctr = 0;


private:

  void runParameterOptimization(int64_t start_ms);
  void runParameterOptimizationGradientDescent(int64_t start_ms);

  // Self state and position information and communication
  bool armed_;

  uint8_t self_sysid_;

  int64_t last_gps_timestamp_ms_ = 0;


  // Kalman filter
  EKFEstimator ekf_;
  EKFParams ekf_params_;

  // History management
  StateHistoryManager shm_;


  OfflineProcessing offline_processing;
  std::string file_name_prefix_;

public:
  mavlink_stream_t* mavlink_stream_;
  global_position_t origin_;

  // Temporary: Simulation / additional analysis.
  std::vector<Vector3> real_positions_;
  float enable_gps_update = 0.0f;

  // For offline analysis in IROS 21 paper
  std::vector<Vector3> wps_to_enable_gps_;
};


// C interface
void init_state_estimation(double origin_lat, double origin_lon, StateEstimation *se);
int state_estimation_read_beacon_position_from_file(void* data);
void state_estimation_send_estimated_position();
void state_estimation_send_detected_beacons_position();
void state_estimation_notify_logfile_eof();
void ceres_test();



#endif /* STATE_ESTIMATION_H */

