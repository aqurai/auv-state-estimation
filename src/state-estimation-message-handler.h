/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   message-handler.h
 * Author: quraishi
 *
 * Created on 13 April, 2019, 7:34 PM
 */

#ifndef STATE_ESTIMATION_MESSAGE_HANDLER_H
#define STATE_ESTIMATION_MESSAGE_HANDLER_H

#include "imu-calibration.h"
#include "state_estimation.h"
#include "Library/communication/mavlink_stream.h"
#include "Library/communication/mavlink_message_handler.h"

void setupStateEstimatorCallabcks(StateEstimation *se, mavlink_message_handler_t *message_handler, uint8_t self_sysid);



#endif /* STATE_ESTIMATION_MESSAGE_HANDLER_H */

