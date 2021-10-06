/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   helper_utils.h
 * Author: anwar
 *
 * Created on April 22, 2020, 1:01 PM
 */

#ifndef HELPER_UTILS_H
#define HELPER_UTILS_H

#include "coord_conventions.h"
#include "Eigen/Eigen"
#include <iostream>

typedef double SEFloatingType;

typedef Eigen::Matrix<SEFloatingType,2,1> Vector2;
typedef Eigen::Matrix<SEFloatingType,3,1> Vector3;
typedef Eigen::Matrix<SEFloatingType,9,1> Vector9;

typedef Eigen::Matrix<SEFloatingType,2,2> Matrix2;
typedef Eigen::Matrix<SEFloatingType,3,3> Matrix3;
typedef Eigen::Matrix<SEFloatingType,9,9> Matrix9;
typedef Eigen::Matrix<SEFloatingType,9,2> Matrix9_2;
typedef Eigen::Matrix<SEFloatingType,9,3> Matrix9_3;

typedef Eigen::AngleAxis<SEFloatingType> AngleAxisType;
typedef Eigen::Quaternion<SEFloatingType> QuatType;


template <typename T> 
T h_sign(T val) {
    return T((T(0) < val) - (val < T(0)));
}

float h_distanceFromGps(global_position_t pos1, global_position_t pos2);

global_position_t h_vectToGps(Vector3 local_pos, global_position_t origin, double heading_deg);

Vector3 h_gpsToVect(global_position_t global_pos, global_position_t origin);

template <typename T> 
void printMatrix(std::string name, T mat, bool suppress=false) {
    Eigen::MatrixXd mm = mat;
    if (suppress) {
        size_t sz = mm.rows() * mm.cols();
        auto d = mm.data();
        for (size_t i=0; i<sz; i++) {
            if (-1e-10 < d[i] && d[i] < 1e-10) {
                d[i] = 0;
            }
        }
    }
    std::cout << name << ":\n" << mm << "\n\n";
}

template <typename T> 
void printVector(std::string name, T vec, bool suppress=false) {
    Eigen::VectorXd vv = vec;
    if (suppress) {
        size_t sz = vv.rows() * vv.cols();
        auto d = vv.data();
        for (size_t i=0; i<sz; i++) {
            if (-1e-10 < vec(i) && vec(i) < 1e-10) {
                d[i] = 0;
            }
        }
    }
    std::cout << name << ": " << vv.transpose() << "\n\n";
}
#endif /* HELPER_UTILS_H */

