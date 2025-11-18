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

#include "sensor/imu.h"

namespace garlileo {
    IMUFrameArray::IMUFrameArray(double timestamp, const std::vector<IMUFrame::Ptr> &frames)
            : _timestamp(timestamp), _frames(frames) {}

    IMUFrameArray::Ptr IMUFrameArray::Create(double timestamp, const std::vector<IMUFrame::Ptr> &frames) {
        return std::make_shared<IMUFrameArray>(timestamp, frames);
    }

    double IMUFrameArray::GetTimestamp() const {
        return _timestamp;
    }

    void IMUFrameArray::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    const std::vector<IMUFrame::Ptr> &IMUFrameArray::GetFrames() const {
        return _frames;
    }

    bool IMUFrameArray::SaveFramesArraysToDisk(const std::string &filename,
                                               const std::vector<IMUFrameArray::Ptr> &arrays,
                                               int precision) {
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(precision);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("imu_arrays", arrays));
        return true;
    }
}
