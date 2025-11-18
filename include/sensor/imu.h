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

#ifndef GARLILEO_IMU_H
#define GARLILEO_IMU_H

// #include "ctraj/core/imu.h"
#include "sensor/imu_frame.h"

namespace garlileo {
    using IMUFrame = ns_imu::IMUFrame;

    struct IMUFrameArray {
    public:
        using Ptr = std::shared_ptr<IMUFrameArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<IMUFrame::Ptr> _frames;

    public:
        explicit IMUFrameArray(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        static IMUFrameArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<IMUFrame::Ptr> &GetFrames() const;

        // save radar frames sequence to disk
        static bool SaveFramesArraysToDisk(const std::string &filename,
                                           const std::vector<IMUFrameArray::Ptr> &arrays,
                                           int precision = 10);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("frames", _frames));
        }
    };
}


#endif
