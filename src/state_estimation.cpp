/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "state_estimation.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <chrono>

#include "plog/Log.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "state-measurement-history.h"
#include "debug_csv_write.h"
#include "covariance_intersection.h"
#include "convex-optimizer.h"

#include "Library/communication/mavlink_stream.h"
#include "Library/hal_emu/drivers/delay.h"
#include "Library/util/helper_utils.h"

StateEstimation * state_estimation;

void temp_mavlink_send_data_stream(mavlink_stream_t* mavlink_stream, int16_t* values, int size, time_microseconds_t timestamp) {
  mavlink_message_t msg;
  int i = 0;
  int number_of_blocks = size / 64;

  int stream_id = 0;
  for (i = 0; i < number_of_blocks; i++) {
    mavlink_msg_raw_data_stream_16_pack(mavlink_stream->sysid, mavlink_stream->compid, &msg, timestamp / 1000, stream_id, number_of_blocks, i, 64, values + i * 64);

    mavlink_stream_send(mavlink_stream, &msg);
    //bytestream_start_transmission(mavlink_stream->tx);
    //bytestream_flush(mavlink_stream->tx);
  }
}

void init_state_estimation(double origin_lat, double origin_lon, StateEstimation *se) {
  global_position_t origin;
  origin.latitude = origin_lat;
  origin.longitude = origin_lon;
  origin.altitude = 0.0f;
  origin.heading = 0.0f;

  state_estimation = se;
}

void state_estimation_send_estimated_position() {
  if (state_estimation != NULL) {
    state_estimation->sendPositionGps();
  }
}

void state_estimation_send_detected_beacons_position() {
  if (state_estimation != NULL) {
    //state_estimation->sendDetectedBeacons();
  }
}

void state_estimation_notify_logfile_eof() {
  LOG_ERROR << "Log eof reached";
  exit(0);
}

// *****************************************************************************

StateEstimation::StateEstimation(global_position_t origin, uint8_t self_sysid, mavlink_stream_t* mavlink_stream, std::string fn_prefix) :
armed_(false),
last_gps_timestamp_ms_(0) {
  origin_ = origin;
  self_sysid_ = self_sysid;
  mavlink_stream_ = mavlink_stream;
  file_name_prefix_ = fn_prefix;
  offline_processing.openFiles(file_name_prefix_);

  //    if (mavlink_stream->sysid == 1) {
  //        StateHistoryManager shm;
  //        shm.insertDataElement(NULL);
  //    }


  // Enable GPS when close to these points
  Vector3 pos_ned;
  global_position_t gps_pos;
  gps_pos.altitude = (double) 0.0f;
  gps_pos.heading = 0.0f;


  gps_pos.latitude = (double) 46.517085656f;
  gps_pos.longitude = (double) 6.584674885f;
  pos_ned = h_gpsToVect(gps_pos, origin_);
  wps_to_enable_gps_.push_back(pos_ned);

  gps_pos.latitude = (double) 46.516892808f;
  gps_pos.longitude = (double) 6.584651198f;
  pos_ned = h_gpsToVect(gps_pos, origin_);
  wps_to_enable_gps_.push_back(pos_ned);

  gps_pos.latitude = (double) 46.516883507f;
  gps_pos.longitude = (double) 6.584838447f;
  pos_ned = h_gpsToVect(gps_pos, origin_);
  wps_to_enable_gps_.push_back(pos_ned);

  gps_pos.latitude = (double) 46.517068398f;
  gps_pos.longitude = (double) 6.584863890f;
  pos_ned = h_gpsToVect(gps_pos, origin_);
  wps_to_enable_gps_.push_back(pos_ned);

  for (auto w : wps_to_enable_gps_) {
    std::cout << "WP:  " << w.transpose() << "\n";
  }
}

void StateEstimation::resetPosition(Vector3 pos) {
  ekf_.reset(pos, time_keeper_get_millis());
}

void StateEstimation::sendPositionGps() {
  sendPositionGps(ekf_.state_.position_, time_keeper_get_millis());
}

void StateEstimation::sendPositionGps(Vector3 position, uint32_t timestamp_ms) {
  // offline_processing.sendProcessedGPSPosition(this, timestamp_ms, position);
  float hdg_deg = h_getYawFromQuat(ekf_.state_.attitude_quat) * 180.0f / M_PI;
  local_coordinates_t local_pos;
  local_pos.pos.v[0] = position(0);
  local_pos.pos.v[1] = position(1);
  local_pos.pos.v[2] = position(2);

  local_pos.heading = hdg_deg;
  local_pos.origin = origin_;

  global_position_t gpos = h_vectToGps(position, origin_, hdg_deg * M_PI / 180.0f);

  if (hdg_deg < 0) {
    hdg_deg += 360.0f;
  }

  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
          &msg,
          timestamp_ms,
          (int32_t) (((double) gpos.latitude) * 10000000),
          (int32_t) (((double) gpos.longitude) * 10000000),
          gpos.altitude * 1000.0f,
          -gpos.altitude * 1000,
          0, // vx 
          0, // vy
          0, // vz
          (uint16_t) hdg_deg // hdg
          );
  mavlink_stream_send(mavlink_stream_, &msg);

  //LOG_WARNING << "GPS Update switch val " << enable_gps_update;


  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "est.lin.acc", time_keeper_get_micros(),
  //            ekf_.state_.acceleration_(0), ekf_.state_.acceleration_(1), ekf_.state_.acceleration_(2));
  //    mavlink_stream_send(mavlink_stream_, &msg);
  //    
  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "pos.cov", time_keeper_get_micros(),
  //            ekf_.state_.covariance_(0,0), ekf_.state_.covariance_(1,1), ekf_.state_.covariance_(2,2));
  //    mavlink_stream_send(mavlink_stream_, &msg);
  //    
  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "vel.cov", time_keeper_get_micros(),
  //            ekf_.state_.covariance_(3,3), ekf_.state_.covariance_(4,4), ekf_.state_.covariance_(5,5));
  //    mavlink_stream_send(mavlink_stream_, &msg);
  //    
  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "est.vel", time_keeper_get_micros(),
  //            ekf_.state_.velocity_(0), ekf_.state_.velocity_(1), ekf_.state_.velocity_(2));
  //    mavlink_stream_send(mavlink_stream_, &msg);
  //    
  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "est.pos", time_keeper_get_micros() / 1000000,
  //            ekf_.state_.position_(0), ekf_.state_.position_(1), ekf_.state_.position_(2));
  //    mavlink_stream_send(mavlink_stream_, &msg);

}

void StateEstimation::receiveMotorCommands(SEFloatingType motor_left, SEFloatingType motor_right, int64_t timestamp_ms) {
  MotorCommandData *mcd = new MotorCommandData(motor_left, motor_right, timestamp_ms);
  shm_.insertDataElement(mcd);
  mcd->updateState(&ekf_, ekf_params_);

  //    ekf_.setMotorSpeed(motor_left, motor_right, timestamp_ms, ekf_params_);

}

void StateEstimation::receiveAttitude(QuatType attitude_quat, int64_t timestamp_ms) {
  AttitudeData * ad = new AttitudeData(attitude_quat, timestamp_ms);
  shm_.insertDataElement(ad);
  ad->updateState(&ekf_, ekf_params_);

  //    ekf_.setAttitude(attitude_quat, ekf_params_);

}

int64_t last_optimization_ms = 0;

void StateEstimation::receiveIMU(Vector3 accelero, Vector3 gyro, int64_t timestamp_ms) {
  IMUData *imud = new IMUData(accelero, gyro, timestamp_ms);
  shm_.insertDataElement(imud);
  imud->updateState(&ekf_, ekf_params_);

  if (timestamp_ms - shm_.getLastStateFrameTimestampMs() > 5000) {
    shm_.insertStateFrame(&ekf_, timestamp_ms);
  }

  //    ekf_.imuUpdate(accelero, gyro, timestamp_ms, ekf_params_);
}

int num_gps_upd = 0;
int num_opt = 0;

void StateEstimation::receiveGPSLocalPosition(Vector3 gpsPosLocal, int64_t timestamp_ms) {
  if (last_gps_timestamp_ms_ == timestamp_ms) {
    return;
  }
  last_gps_timestamp_ms_ = timestamp_ms;


#ifndef OFFLINE_GPS_OPT
  // If using GPS update directly.
  // Usually GPS is not used but recorded for ground truth.
  if (enable_gps_update > 0.5f) {
    GPSLocalData *gpsd = new GPSLocalData(gpsPosLocal, timestamp_ms);
    shm_.insertDataElement(gpsd);
    gpsd->updateState(&ekf_, ekf_params_);
  } else {
    ekf_.predictStep(timestamp_ms, ekf_params_);
  }
  offline_processing.writeState(ekf_.state_.position_, ekf_.state_.covariance_.block<3, 3>(0, 0), ekf_.state_.timestamp_ms_);
#endif

#ifdef OFFLINE_PROCESSING
  GPSLocalData *gpsd = new GPSLocalData(gpsPosLocal, timestamp_ms);
  shm_.insertDataElement(gpsd);

  if (timestamp_ms - last_optimization_ms > 20000) {
    runParameterOptimizationGradientDescent(timestamp_ms - 5000);
    last_optimization_ms = timestamp_ms;
    num_opt++;
  }

  // Do GPS update near specific waypoints.
  bool near_wp = false;
  for (auto w : wps_to_enable_gps_) {
    w(2) = 0;
    gpsPosLocal(2) = 0;
    double d = (w - gpsPosLocal).norm();
    //LOG_WARNING << "-             --------- >>>>> Dist to WP (" << w(0) << ", " << w(1) << ")    is  " << d;
    if (d < 2.0f) {
      near_wp = true;
      LOG_WARNING << "-             --------- >>>>> Dist to WP (" << w(0) << ", " << w(1) << ")    is  " << d;
      break;
    }
  }

  // Perform GPS update if:
  //  It is enabled by setting
  //  In the initial part of the trajectory (Coinciding starting point for comparing true and estimated traj)
  //  The robot is near specific waypoints.
  if (enable_gps_update > 0.5f || num_gps_upd < 30 || near_wp) {
    num_gps_upd++;
    gpsd->updateState(&ekf_, ekf_params_);
    offline_processing.writeGPSUpdatePositions(gpsPosLocal, timestamp_ms);
  } else {
    ekf_.predictStep(timestamp_ms, ekf_params_);
  }

#endif

  offline_processing.iros_2021_write_gps(gpsPosLocal, timestamp_ms);


}

void StateEstimation::receiveRangeMeasurement(uint8_t beacon_sys_id, Vector3 beacon_pos, Matrix2 beacon_cov, SEFloatingType range_m, int64_t timestamp_ms) {
  beacon_pos(2) = 0.0f;

  offline_processing.writeState(ekf_.state_.position_, ekf_.state_.covariance_.block<3, 3>(0, 0), ekf_.state_.timestamp_ms_);
  offline_processing.writeRangeMeasurement(beacon_sys_id, beacon_pos, beacon_cov, range_m, timestamp_ms);


#ifdef RANGING_BEACONS
  AcousticRangeData *acd = new AcousticRangeData(beacon_sys_id, beacon_pos, beacon_cov, range_m, timestamp_ms);
  shm_.insertDataElement(acd);
  acd->updateState(&ekf_, ekf_params_);
  runParameterOptimizationGradientDescent(timestamp_ms - 5000);
#endif

#ifdef COOP_LOC    

  ekf_.predictStep(timestamp_ms);
  auto [posr, covar] =
          rangeBasedCovarianceIntersection(ekf_.state_.state_, ekf_.state_.covariance_, beacon_pos, beacon_cov, range_m);

  ekf_.state_.state_ = posr;
  ekf_.state_.covariance_ = covar;
#endif

  offline_processing.writeState(ekf_.state_.position_, ekf_.state_.covariance_.block<3, 3>(0, 0), ekf_.state_.timestamp_ms_);
  LOG_WARNING << (int) timestamp_ms << ": Range Update";
}

void StateEstimation::receiveDepth(SEFloatingType depth_m, SEFloatingType depth_vel, int64_t timestamp_ms) {
  DepthSensorData *ds = new DepthSensorData(depth_m, depth_vel, timestamp_ms);
  shm_.insertDataElement(ds);
  ds->updateState(&ekf_, ekf_params_);

  offline_processing.writeState(ekf_.state_.position_, ekf_.state_.covariance_.block<3, 3>(0, 0), ekf_.state_.timestamp_ms_);

}

void StateEstimation::receiveRangeBearingMeasurement(Vector3 pos_2d, Matrix2 cov, int64_t timestamp_ms) {
  AcousticRangeBearingData *arbd = new AcousticRangeBearingData(pos_2d, cov, timestamp_ms);
  shm_.insertDataElement(arbd);
  arbd->updateState(&ekf_, ekf_params_);

  runParameterOptimizationGradientDescent(timestamp_ms - 15000);
}

void StateEstimation::receiveBeaconNavData(uint8_t beacon_sysid, Vector3 position, Matrix2 covariance, SEFloatingType heading_deg, SEFloatingType range_m, uint8_t to_sysid, time_milliseconds_t t_ms) {
  // Moved to beacon manager
}

uint8_t StateEstimation::getBestBeaconId() {
  // Moved to beacon manager
}

void StateEstimation::updateOrigin(global_position_t& global_origin) {
  if (global_origin.latitude != origin_.latitude || global_origin.longitude != origin_.longitude) {
    LOG_WARNING << "SE: Updating origin ";
  }
  origin_ = global_origin;
}

void StateEstimation::receiveModeByte(uint8_t base_mode) {
  bool armed = base_mode >> 7;
  if (!armed_ && armed) {
    // reset;
    LOG_WARNING << "Re-armed, resetting filter.";

    // resetFilter();

  }
  armed_ = armed;
}

void StateEstimation::receiveIMUPreintegrated(Vector3 position, Eigen::Matrix3f covariance, int64_t timestamp_ms) {
  //    position_queue_.emplace(PreIntegratedPosition(position, covariance, timestamp_ms));
  //  PosNode p(position, covariance, timestamp_ms);
  //ekf_.addNode(p);
}

void StateEstimation::updateBeaconPositionNEDAcoustics(uint8_t sys_id, double x, double y, int64_t timestamp_ms, int16_t *bitscores) {

}

Vector3 StateEstimation::getPosition() {
  Vector3 position = ekf_.state_.position_;
  return position;
}

Matrix3 StateEstimation::getPositionCovariance() {
  Matrix3 cov;
  cov = ekf_.state_.covariance_.block<3, 3>(0, 0);
  return cov;
}

SEFloatingType StateEstimation::getYaw() {
  QuatType q = ekf_.state_.attitude_quat;
  return atan2(2 * (q.w() * q.z() + q.x() * q.y()), (q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
}

QuatType StateEstimation::getAttitude() {
  return ekf_.state_.attitude_quat;
}

void StateEstimation::runParameterOptimization(int64_t start_ms) {
  /*
  std::cout << "\n\n======================================================\n";
  std::cout << "Heading offset optimize \n";
  auto start = std::chrono::system_clock::now();
    
  SEFloatingType heading_err_est = 0.0f;
  SEFloatingType min_err = 1000000;
  EKFParams p;
  p.param_hdg_offset_deg = heading_err_est;
  size_t num_tests = 0;
    
  for (double h = -180; h < 180.0f; h=h+1.0f) {
      p.param_hdg_offset_deg = h;
      SEFloatingType e = shm_.evaluateChainUpdateError(start_ms, p);
        
      if (e < min_err) {
          min_err = e;
          heading_err_est = h;
      }
      num_tests++;
      //std::cout << "Tested for value " << h << "deg, Error = " << e << "\n";
  }
    
  auto end = std::chrono::system_clock::now();
  auto d_milli = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
  std::cout << (int)num_tests << " Values tested. Time taken: " << d_milli << "ms \n";
    
  LOG_WARNING << "Estimated heading err deg = " << heading_err_est;
  std::cout << "======================================================\n";
    
  ekf_params_.param_hdg_offset_deg = heading_err_est;
    
  mavlink_message_t msg;
  mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
          &msg, "hdg,ms,num", time_keeper_get_micros(),
          heading_err_est, d_milli, num_tests);
  mavlink_stream_send(mavlink_stream_, &msg);
    
  //temp_mavlink_send_data_stream(mavlink_stream_, errvals, 360,time_keeper_get_micros());
   */
}

void StateEstimation::runParameterOptimizationGradientDescent(int64_t start_ms) {
  //std::cout << "\n\n======================================================\n";
  //std::cout << "Gradient descent Heading offset optimize \n";
  auto start = std::chrono::system_clock::now();

  SEFloatingType param_block[6] = {ekf_params_.param_hdg_offset_deg,
    ekf_params_.param_thrust_K,
    ekf_params_.param_drag_coef_X,
    ekf_params_.param_drag_coef_Y,
    ekf_params_.param_stream_vel_x,
    ekf_params_.param_stream_vel_y};
  SEFloatingType *res = parameterOptimize(&shm_, start_ms, param_block);


  ekf_params_.param_hdg_offset_deg = res[0];
  ekf_params_.param_thrust_K = res[1];
  ekf_params_.param_drag_coef_X = res[2];
  ekf_params_.param_drag_coef_Y = res[3];
  ekf_params_.param_stream_vel_x = res[4];
  ekf_params_.param_stream_vel_y = res[5];

  mavlink_message_t msg;
  //    mavlink_msg_debug_vect_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
  //            &msg, "hdg,ms,num", time_keeper_get_micros(),
  //            heading_err_est, d_milli, num_iters);


  mavlink_msg_imu_calib_params_pack(mavlink_stream_->sysid, mavlink_stream_->compid,
          &msg, time_keeper_get_millis(), 0,
          (float) res[0], (float) res[1], (float) res[2],
          (float) res[3], (float) res[4], 0);
  mavlink_stream_send(mavlink_stream_, &msg);

}

void StateEstimation::initializeStartingPosition(Vector3 gpsPosLocal, int64_t timestamp_ms) {
  if (gps_init_time_ctr < 10) {
    gps_init_time_ctr++;
    ekf_.reset(gpsPosLocal, time_keeper_get_millis());
    shm_.reset();
  }
}

