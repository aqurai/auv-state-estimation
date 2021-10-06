/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   convex-optimizer.h
 * Author: anwar
 *
 * Created on January 27, 2021, 9:48 PM
 */

#ifndef CONVEX_OPTIMIZER_H
#define CONVEX_OPTIMIZER_H

class StateHistoryManager;

#include <stdint.h>

#include "ekf-estimator.h"

double *parameterOptimize(StateHistoryManager *shm, int64_t start_ms, double * param_block);


#endif /* CONVEX_OPTIMIZER_H */

