#ifndef _MATRIX_DEFINITIONS_H
#define _MATRIX_DEFINITIONS_H

#include <Eigen/Dense>

constexpr int stateN = 4;
constexpr int controlM = 4;
constexpr int measureK = 2;

typedef Eigen::Matrix<float,stateN,stateN> MatrixAGf;

typedef Eigen::Matrix<float,stateN,controlM> MatrixBf;

typedef Eigen::Matrix<float,stateN,stateN> MatrixRf;

typedef Eigen::Matrix<float,stateN,1> VectorMuf;

typedef Eigen::Matrix<float,stateN,stateN> MatrixSigf;

typedef Eigen::Matrix<float,controlM,1> VectorUtf;

typedef Eigen::Matrix<float,measureK,1> VectorZtf;

typedef Eigen::Matrix<float,measureK,stateN> MatrixCHf;

typedef Eigen::Matrix<float,stateN,measureK> MatrixKf;

typedef Eigen::Matrix<float,measureK,measureK> MatrixQf;

#endif