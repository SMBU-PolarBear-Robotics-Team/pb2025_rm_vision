// Copyright 2022 ChenJun
// Copyright 2024 Zheng Yu
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_
#define ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  // Set the initial state
  void setState(const Eigen::VectorXd & x0);

  // Compute a predicted state
  Eigen::MatrixXd predict();

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  // NOLINTBEGIN
  // Process nonlinear vector function
  VecVecFunc f;
  // Observation nonlinear vector function
  VecVecFunc h;
  // Jacobian of f()
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  // Jacobian of h()
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  // Process noise covariance matrix
  VecMatFunc update_Q;
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Priori state
  Eigen::VectorXd x_pri;
  // Posteriori state
  Eigen::VectorXd x_post;
  // NOLINTEND
};

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_
