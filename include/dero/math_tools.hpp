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

// Reference:
// https://github.com/lastflowers/envio/blob/master/src/utils/Attitude.h

#ifndef MATH_TOOLS_HPP
#define MATH_TOOLS_HPP

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <dero/nav_convert.hpp>
#include <dero/variable_define.hpp>

namespace incsl {

Mat3d  skewMatrix(Vec3d &vec);
Mat12d EnsurePSDDr(Mat12d &matrix);
Mat15d EnsurePSD(Mat15d &matrix);
Mat18d EnsurePSDCloning(Mat18d &matrix);
Vec4d  quatMultiplication(Vec4d &q1, Vec4d &q2);
Mat4d  calculateOmega(Vec3d &w);
Mat4d  calculateLeftOmega(const Vec4d &v);
double wrapTo2Pi(const double &angle_);
} // namespace incsl

#endif // MATH_TOOLS_HPP
