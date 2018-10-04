/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SD_SLAM_SENSOR_H_
#define SD_SLAM_SENSOR_H_

#define SQUARE(a) ((a)*(a))

#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace SD_SLAM {

class Sensor {
 public:
  Sensor();
  ~Sensor();

  inline int GetStateSize() { return state_size_; }
  inline int GetMeasurementSize() { return measurement_size_; }

  inline void SetLastPose(const Eigen::Matrix4d &pose) {
    last_pose_ = pose;
  }

  // Return current pose calculated from sensor
  virtual Eigen::Matrix4d GetPose(const Eigen::VectorXd &X);

  // Get input pose and convert to position and quaternion
  virtual Eigen::VectorXd PoseToVector(const Eigen::Matrix4d &pose);

  // Init sensor with default values
  virtual void Init(Eigen::VectorXd &X, Eigen::MatrixXd &P) = 0;

  // Set initial state
  virtual void InitState(Eigen::VectorXd &X, const Eigen::VectorXd &Z) = 0;

  // Predict state
  virtual void F(Eigen::VectorXd &X, double time) = 0;

  // Prediction jacobian
  virtual Eigen::MatrixXd jF(const Eigen::VectorXd &X, double time) = 0;

  // Process noise covariance
  virtual Eigen::MatrixXd Q(const Eigen::VectorXd &X, double time) = 0;

  // Get measurement
  virtual Eigen::VectorXd Z(const Eigen::Matrix4d &pose, const std::vector<double> &params, double time) = 0;

  // Get predicted measurement
  virtual Eigen::VectorXd H(const Eigen::VectorXd &X, double time) = 0;

  // Measurement jacobian
  virtual Eigen::MatrixXd jH(const Eigen::VectorXd &X, double time) = 0;

  // Measurement noise covariance
  virtual Eigen::MatrixXd R(const Eigen::VectorXd &X, double time) = 0;

 protected:
  // Calculate quaternion from angular velocity
  Eigen::Quaterniond QuaternionFromAngularVelocity(const Eigen::Vector3d &w);

  // Quaternion jacobian
  Eigen::Matrix4d QuaternionJacobian(const Eigen::Quaterniond &q);
  Eigen::Matrix4d QuaternionJacobianRight(const Eigen::Quaterniond &q);

  // Jacobian dw/dq
  Eigen::Matrix<double, 4, 3> dq_by_dw(const Eigen::Quaterniond &q, const Eigen::Vector3d &w, double time);

  int state_size_;
  int measurement_size_;

  // Pose information
  Eigen::Matrix4d last_pose_;

  // Covariance
  static const double COV_X_2;
  static const double COV_Q_2;
  static const double COV_V_2;
  static const double COV_W_2;

  // Noise
  static const double SIGMA_X;
  static const double SIGMA_Q;
  static const double SIGMA_V;
  static const double SIGMA_W;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_SENSOR_H_
