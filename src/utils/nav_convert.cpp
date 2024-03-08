// This file is part of DeRO: Dead Reckoning Based on Radar Odometry
// With Accelerometers Aided for Robot Localization.
// Copyright (C) 2024 Hoang Viet Do  <hoangvietdo@sju.ac.kr>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// Reference: https://github.com/lastflowers/envio/blob/master/src/utils/Attitude.cpp

#include "dero/nav_convert.hpp"

// TODO: Add explaination and reference (equation) for each function

using namespace Eigen;

namespace incsl {

Vec4d euler2quat(Vec3d &euler) {
  double phi   = euler(0);
  double theta = euler(1);
  double psi   = euler(2);

  Vec4d quat = Vec4d::Zero();

  double cs = std::cos(psi / 2.0);
  double ss = std::sin(psi / 2.0);
  double ct = std::cos(theta / 2.0);
  double st = std::sin(theta / 2.0);
  double cp = std::cos(phi / 2.0);
  double sp = std::sin(phi / 2.0);

  double a = cp * ct * cs + sp * st * ss;
  double b = sp * ct * cs - cp * st * ss;
  double c = cp * st * cs + sp * ct * ss;
  double d = cp * ct * ss - sp * st * cs;

  quat(0, 0) = a;
  quat(1, 0) = b;
  quat(2, 0) = c;
  quat(3, 0) = d;

  quat = quatNormalize(quat);

  return quat;
} // euler2quat

Mat3d euler2dcm(Vec3d &euler) {
  double phi   = euler(0);
  double theta = euler(1);
  double psi   = euler(2);

  double cs = std::cos(psi);
  double ss = std::sin(psi);
  double ct = std::cos(theta);
  double st = std::sin(theta);
  double cp = std::cos(phi);
  double sp = std::sin(phi);

  Mat3d Cx, Cy, Cz;
  Cx << cs, ss, 0.0, -ss, cs, 0.0, 0.0, 0.0, 1.0;

  Cy << ct, 0.0, -st, 0.0, 1.0, 0.0, st, 0.0, ct;

  Cz << 1.0, 0.0, 0.0, 0.0, cp, sp, 0.0, -sp, cp;

  Mat3d dcm = Mat3d::Zero();
  dcm       = Cx.transpose() * Cy.transpose() * Cz.transpose();
  // dcm = dcmNormalize(dcm);

  return dcm;
} // euler2dcm

Vec4d dcm2quat(Mat3d &dcm) {
  Vec4d quat = Vec4d::Zero();

  quat(0, 0) = std::sqrt(1.0 / 4.0 * (1.0 + dcm(0, 0) + dcm(1, 1) + dcm(2, 2)));
  quat(1, 0) = std::sqrt(1.0 / 4.0 * (1.0 + dcm(0, 0) - dcm(1, 1) - dcm(2, 2)));
  quat(2, 0) = std::sqrt(1.0 / 4.0 * (1.0 - dcm(0, 0) + dcm(1, 1) - dcm(2, 2)));
  quat(3, 0) = std::sqrt(1.0 / 4.0 * (1.0 - dcm(0, 0) - dcm(1, 1) + dcm(2, 2)));

  int max_idx = 1;
  for (int i = 1; i <= 3; i++) {
    if (quat(i, 0) > quat(max_idx, 0)) {
      max_idx = i;
    } // if
  }   // for

  if (max_idx == 0) {
    quat(1, 0) = (dcm(2, 1) - dcm(1, 2)) / 4.0 / quat(0, 0);
    quat(2, 0) = (dcm(0, 2) - dcm(2, 0)) / 4.0 / quat(0, 0);
    quat(3, 0) = (dcm(1, 0) - dcm(0, 1)) / 4.0 / quat(0, 0);

  } else if (max_idx == 1) {
    quat(0, 0) = (dcm(2, 1) - dcm(1, 2)) / 4.0 / quat(1, 0);
    quat(2, 0) = (dcm(1, 0) - dcm(0, 1)) / 4.0 / quat(1, 0);
    quat(3, 0) = (dcm(0, 2) - dcm(2, 0)) / 4.0 / quat(1, 0);

  } else if (max_idx == 2) {
    quat(0, 0) = (dcm(0, 2) - dcm(2, 0)) / 4.0 / quat(2, 0);
    quat(1, 0) = (dcm(1, 0) - dcm(0, 1)) / 4.0 / quat(2, 0);
    quat(3, 0) = (dcm(2, 1) - dcm(1, 2)) / 4.0 / quat(2, 0);

  } else if (max_idx == 3) {
    quat(0, 0) = (dcm(1, 0) - dcm(0, 1)) / 4.0 / quat(3, 0);
    quat(1, 0) = (dcm(0, 2) - dcm(2, 0)) / 4.0 / quat(3, 0);
    quat(2, 0) = (dcm(2, 1) - dcm(1, 2)) / 4.0 / quat(3, 0);

  } // if

  if (quat(0, 0) < 0.0) {
    quat = -quat;
  } // if

  quat = quatNormalize(quat);

  return quat;
} // dcm2quat

Vec3d dcm2euler(Mat3d &dcm) {
  Vec3d  euler = Vec3d::Zero();
  double phi, theta, psi;

  theta = std::atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

  if (dcm(2, 0) <= -0.999) {
    phi = sqrt(-1); // NaN
    psi = std::atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
  } else if (dcm(2, 0) >= 0.999) {
    phi = sqrt(-1); // NaN
    psi = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
  } else {
    phi = std::atan2(dcm(2, 1), dcm(2, 2));
    psi = std::atan2(dcm(1, 0), dcm(0, 0));
  } // if

  euler(0, 0) = phi;
  euler(1, 0) = theta;
  euler(2, 0) = psi;

  return euler;
} // dcm2euler

Vec3d quat2euler(Vec4d &quat) {
  Vec3d euler = Vec3d::Zero();
  Mat3d foo   = Mat3d::Zero();

  foo   = quat2dcm(quat);
  euler = dcm2euler(foo);

  return euler;
} // quat2euler

Mat3d quat2dcm(Vec4d &quat) {
  Mat3d  dcm = Mat3d::Zero();
  double a   = quat(0, 0);
  Vec3d  vec;

  vec << quat(1, 0), quat(2, 0), quat(3, 0);
  dcm = (a * a - vec.norm() * vec.norm()) * Mat3d::Identity() + 2.0 * a * skewMatrix(vec) + 2.0 * vec * vec.transpose();
  dcm = dcmNormalize(dcm);

  return dcm;
} // quat2dcm

Vec4d quatNormalize(Vec4d &quat) {
  Vec4d quat_ = Vec4d::Zero();

  quat_ = quat / quat.norm();

  return quat_;
} // quatNormalize

Mat3d dcmNormalize(Mat3d &dcm) {
  Mat3d dcm_ = Mat3d::Zero();

  JacobiSVD<Mat3d> svd(dcm, ComputeFullU | ComputeFullV);
  dcm_ = svd.matrixU() * Mat3d::Identity() * svd.matrixV().transpose();

  return dcm_;
} // dcmNormalize

} // namespace incsl
