// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef FOOT_H
#define FOOT_H

#include "filesystem"
#include "fstream"
#include "memory"
#include "ostream"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/eigen_utils.hpp"
#include "utility"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_foot {

struct FootFrame {
public:
    using Ptr = std::shared_ptr<FootFrame>;

private:
    double _timestamp;
    Eigen::Vector4d _contact;
    bool _contact_event; //foot contact가 일어난 직후인지 판단하는 variable
    int _contact_duration;


public:
    explicit FootFrame(double timestamp = INVALID_TIME_STAMP,
                      Eigen::Vector4d contact = Eigen::Vector4d::Ones(),
                      bool contact_event = false,
                      int contact_duration = 0);

    static FootFrame::Ptr Create(double timestamp = INVALID_TIME_STAMP,
                      Eigen::Vector4d contact = Eigen::Vector4d::Ones(),
                      bool contact_event = false,
                      int contact_duration = 0);

    [[nodiscard]] double GetTimestamp() const;

    [[nodiscard]] const Eigen::Vector4d &GetContactLegs() const;

    [[nodiscard]] const bool &GetContactEvent() const;

    [[nodiscard]] const int &GetContactDuration() const;

    void SetTimestamp(double timestamp);

    friend std::ostream &operator<<(std::ostream &os, const FootFrame &frame);

    static bool SaveFramesToDisk(const std::string &filename,
                                 const std::vector<FootFrame::Ptr> &frames,
                                 int precision = 10);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    template <class Archive>
    void serialize(Archive &ar) {
        ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("ContactLeg", _contact), cereal::make_nvp("ContactEvent", _contact_event), cereal::make_nvp("ContactDuration", _contact_duration));
    }
};

} 

#endif
