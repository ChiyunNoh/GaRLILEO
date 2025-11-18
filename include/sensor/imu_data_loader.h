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

#ifndef GARLILEO_IMU_DATA_LOADER_H
#define GARLILEO_IMU_DATA_LOADER_H

#include "sensor_msgs/msg/imu.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor/imu.h"
#include <rclcpp/rclcpp.hpp>



namespace garlileo {
    enum class IMUMsgType {
        SENSOR_IMU,
        SBG_IMU
    };

    class IMUDataUnpacker {
    public:
        using Ptr = std::shared_ptr<IMUDataUnpacker>;

    public:
        explicit IMUDataUnpacker() = default;

        static IMUDataUnpacker::Ptr Create();

        static IMUFrame::Ptr Unpack(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    };

}


#endif 
