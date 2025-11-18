// GaRLILEO: Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024
//   School of Geodesy and Geomatics (SGG), Wuhan University, China
//   Based on: "River: A Tightly-Coupled Radar-Inertial Velocity Estimator
//              Based on Continuous-Time Optimization"
//   Upstream: https://github.com/Unsigned-Long/River
//   Author:   Shuolong Chen
//
// Copyright (c) 2025
//   Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
//
// See LICENSE for the full MIT License text.

#include "sensor/imu_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "spdlog/fmt/fmt.h"

namespace garlileo {


    IMUDataUnpacker::Ptr IMUDataUnpacker::Create() {
        return std::make_shared<IMUDataUnpacker>();
    }

    IMUFrame::Ptr IMUDataUnpacker::Unpack(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {

        auto acce = Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
        );
        auto gyro = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
        );

        auto orientation = Eigen::Quaternion<double>(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z
        );

        return IMUFrame::Create(rclcpp::Time(msg->header.stamp).seconds(), gyro, acce, orientation); 
    }

}