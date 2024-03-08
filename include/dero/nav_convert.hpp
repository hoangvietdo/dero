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

#ifndef NAV_CONVERT_HPP
#define NAV_CONVERT_HPP

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <dero/variable_define.hpp>

namespace incsl {

#define D2R (double)(M_PI / 180.0)
#define R2D (double)(180.0 / M_PI)

Vec4d euler2quat(Vec3d &euler);
Mat3d euler2dcm(Vec3d &euler);

Vec4d dcm2quat(Mat3d &dcm);
Vec3d dcm2euler(Mat3d &dcm);

Vec3d quat2euler(Vec4d &quat);
Mat3d quat2dcm(Vec4d &quat);

Vec4d quatNormalize(Vec4d &quat);
Mat3d dcmNormalize(Mat3d &dcm);
Mat3d skewMatrix(Vec3d &vec);
} // namespace incsl

#endif // NAV_CONVERT_HPP
