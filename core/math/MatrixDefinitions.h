#ifndef _MATRIX_DEFINITIONS_H
#define _MATRIX_DEFINITIONS_H

#include <Eigen/Dense>




// template <typename T, size_t N>
// inline constexpr size_t dim(T(&)[N])
// {
//     return N;
// }

// template <typename T> 
// inline constexpr T get_val(T v) {
//   return v;
// }


constexpr int stateN = 4;
constexpr int controlM = 4;
constexpr int measureK = 2;



typedef Eigen::Matrix<float,stateN,stateN> MatrixAf;

typedef Eigen::Matrix<float,stateN,controlM> MatrixBf;

typedef Eigen::Matrix<float,stateN,stateN> MatrixRf;

typedef Eigen::Matrix<float,stateN,1> VectorMuf;

typedef Eigen::Matrix<float,stateN,stateN> MatrixSigf;

typedef Eigen::Matrix<float,controlM,1> VectorUtf;

typedef Eigen::Matrix<float,measureK,1> VectorZtf;

typedef Eigen::Matrix<float,measureK,stateN> MatrixCf;

typedef Eigen::Matrix<float,stateN,measureK> MatrixKf;

typedef Eigen::Matrix<float,measureK,measureK> MatrixQf;

#endif