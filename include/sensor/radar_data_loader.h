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

#ifndef GARLILEO_RADAR_DATA_LOADER_H
#define GARLILEO_RADAR_DATA_LOADER_H

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor/radar.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

struct EIGEN_ALIGN16 RadarTarget {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float intensity;         // CFAR cell to side noise ratio in [dB]
    float velocity;  // Doppler's velocity in [m/s]

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        RadarTarget,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity)
)
using RadarPointCloud = pcl::PointCloud<RadarTarget>;

namespace garlileo {
    enum class RadarMsgType {
        GARLILEO
    };

    class RadarDataUnpacker {
    public:
        using Ptr = std::shared_ptr<RadarDataUnpacker>;

    public:
        explicit RadarDataUnpacker() = default;

        static RadarDataUnpacker::Ptr Create();

        static RadarTargetArray::Ptr Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    };
}

#endif //GARLILEO_RADAR_DATA_LOADER_H
