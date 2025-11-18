// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef LEG_H
#define LEG_H

#include "filesystem"
#include "fstream"
#include "memory"
#include "ostream"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/eigen_utils.hpp"
#include "utility"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_leg {

struct LegFrame {
public:
    using Ptr = std::shared_ptr<LegFrame>;

private:
    double _timestamp;
    Eigen::Vector3d _linearLegVel;


public:
    explicit LegFrame(double timestamp = INVALID_TIME_STAMP,
                      Eigen::Vector3d linearLegVel = Eigen::Vector3d::Zero());

    static LegFrame::Ptr Create(double timestamp = INVALID_TIME_STAMP,
                                const Eigen::Vector3d &linearLegVel = Eigen::Vector3d::Zero());

    [[nodiscard]] double GetTimestamp() const;

    [[nodiscard]] const Eigen::Vector3d &GetVel() const;
    void SetVel(Eigen::Vector3d &linearLegVel);


    void SetTimestamp(double timestamp);

    friend std::ostream &operator<<(std::ostream &os, const LegFrame &frame);

    static bool SaveFramesToDisk(const std::string &filename,
                                 const std::vector<LegFrame::Ptr> &frames,
                                 int precision = 10);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    template <class Archive>
    void serialize(Archive &ar) {
        ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("linear_vel", _linearLegVel));
    }
};

} 

#endif
