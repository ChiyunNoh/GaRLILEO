// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#include "sensor/leg_data_loader.h"

namespace garlileo {
    LegDataUnpacker::Ptr LegDataUnpacker::Create() {
        return std::make_shared<LegDataUnpacker>();
    }

    LegFrame::Ptr LegDataUnpacker::Unpack(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {

        auto linearLegVel = Eigen::Vector3d(
                msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z
        );     
        const double t = rclcpp::Time(msg->header.stamp).seconds();
        return LegFrame::Create(t, linearLegVel);
    }
}