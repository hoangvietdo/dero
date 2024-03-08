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

#include "dero/math_tools.hpp"

using namespace Eigen;

namespace incsl {

Mat3d skewMatrix(Vec3d &vec) {
  Mat3d Matrix;

  Matrix << 0.0, -vec(2, 0), vec(1, 0), vec(2, 0), 0.0, -vec(0, 0), -vec(1, 0), vec(0, 0), 0.0;
  return Matrix;
} // skewMatrix

Mat15d EnsurePSD(Mat15d &matrix) { return (matrix + matrix.transpose()) / 2.0; } // EnsurePSD

Mat12d EnsurePSDDr(Mat12d &matrix) { return (matrix + matrix.transpose()) / 2.0; } // EnsurePSDDr

Mat18d EnsurePSDCloning(Mat18d &matrix) { return (matrix + matrix.transpose()) / 2.0; } // EnsurePSDCloning

Vec4d quatMultiplication(Vec4d &p, Vec4d &q) {
  double p_w = p(0, 0);
  double p_x = p(1, 0);
  double p_y = p(2, 0);
  double p_z = p(3, 0);

  double q_w = q(0, 0);
  double q_x = q(1, 0);
  double q_y = q(2, 0);
  double q_z = q(3, 0);

  Vec4d quaternion;
  quaternion << p_w * q_w - p_x * q_x - p_y * q_y - p_z * q_z, p_w * q_x + p_x * q_w + p_y * q_z - p_z * q_y,
      p_w * q_y - p_x * q_z + p_y * q_w + p_z * q_x, p_w * q_z + p_x * q_y - p_y * q_x + p_z * q_w;
  quaternion = quatNormalize(quaternion);

  return quaternion;
} // quatMultiplication

Mat4d calculateOmega(Vec3d &w) {
  Mat4d Omega;
  Omega << 0.0, -w(0), -w(1), -w(2), w(0), 0.0, w(2), -w(1), w(1), -w(2), 0.0, w(0), w(2), w(1), -w(0), 0.0;
  return Omega;
} // calculateOmega

Mat4d calculateLeftOmega(const Vec4d &v) {
  Mat4d matrix;
  matrix << v(0), -v(1), -v(2), -v(3), v(1), v(0), -v(3), v(2), v(2), v(3), v(0), -v(1), v(3), -v(2), v(1), v(0);
  return matrix;
} // calculateLeftOmega

double wrapTo2Pi(const double &angle_) {
  double angle;
  bool   was_pos = angle_ > 0;
  angle          = fmod(angle_, 2.0 * M_PI);

  if (was_pos)
    angle = 2.0 * M_PI;
  else if (angle == 0)
    angle = 2.0 * M_PI;
  return angle;
} // wrapTo2Pi

} // namespace incsl
