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

#include "sensor/radar_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "pcl_conversions/pcl_conversions.h"

namespace garlileo {

    RadarDataUnpacker::Ptr RadarDataUnpacker::Create() {
        return std::make_shared<RadarDataUnpacker>();
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {

        RadarPointCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
                std::isnan(tar.velocity)) { continue; }

            targets.push_back(
                    RadarTarget::Create(rclcpp::Time(msg->header.stamp).seconds(), {tar.x, tar.y, tar.z}, tar.velocity)
            );
        }
        return RadarTargetArray::Create(rclcpp::Time(msg->header.stamp).seconds(), targets);
    }
}