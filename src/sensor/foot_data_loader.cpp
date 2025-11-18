// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#include "sensor/foot_data_loader.h"

namespace garlileo {
    FootDataUnpacker::Ptr FootDataUnpacker::Create() {
        return std::make_shared<FootDataUnpacker>();
    }

    FootFrame::Ptr FootDataUnpacker::Unpack(const spot_msgs::msg::FootStateArray::ConstSharedPtr& foot_msg) {

        auto contact = Eigen::Vector4d(
                foot_msg->states[0].contact,
                foot_msg->states[1].contact,
                foot_msg->states[2].contact,
                foot_msg->states[3].contact
        );     

        return FootFrame::Create(rclcpp::Time(foot_msg->header.stamp).seconds(), contact, false, 0); 
    }
}