#ifndef CONTROLLERS_OSC_TYPES_HPP
#define CONTROLLERS_OSC_TYPES_HPP

#include "eigen3/Eigen/Core"

namespace controller {

typedef double Scalar;

typedef int Index;
typedef int Dimension;
typedef int Size;

typedef Eigen::VectorX<Scalar> ConfigurationVector;
typedef Eigen::VectorX<Scalar> TangentVector;
typedef Eigen::VectorX<Scalar> ActuationVector;

// Generic Vector
typedef Eigen::VectorX<Scalar> Vector;
typedef Eigen::Vector3<Scalar> Vector3;

typedef Eigen::MatrixX<Scalar> Matrix;
typedef Eigen::Matrix3<Scalar> Matrix3;

}  // namespace controller

#endif /* CONTROLLERS_OSC_TYPES_HPP */
