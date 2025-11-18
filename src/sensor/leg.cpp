// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#include "sensor/leg.h"

namespace ns_leg {

LegFrame::LegFrame(double timestamp, Eigen::Vector3d linearLegVel)
    : _timestamp(timestamp),
      _linearLegVel(std::move(linearLegVel)) {}

LegFrame::Ptr LegFrame::Create(double timestamp,
                               const Eigen::Vector3d &linearLegVel) {
    return std::make_shared<LegFrame>(timestamp, linearLegVel);
}

double LegFrame::GetTimestamp() const { return _timestamp; }

const Eigen::Vector3d &LegFrame::GetVel() const { return _linearLegVel; }

void LegFrame::SetVel(Eigen::Vector3d &linearLegVel){this->_linearLegVel = linearLegVel;}

std::ostream &operator<<(std::ostream &os, const LegFrame &frame) {
    os << "timestamp: " << frame._timestamp << ", linear vel: " << frame._linearLegVel.transpose();
    return os;
}

void LegFrame::SetTimestamp(double timestamp) { _timestamp = timestamp; }

bool LegFrame::SaveFramesToDisk(const std::string &filename,
                                const std::vector<LegFrame::Ptr> &frames,
                                int precision) {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(precision);
    cereal::JSONOutputArchive ar(file);
    ar(cereal::make_nvp("leg_frames", frames));
    return true;
}

}
