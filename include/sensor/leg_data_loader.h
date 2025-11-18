// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef LEG_DATA_LOADER_H
#define LEG_DATA_LOADER_H

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "sensor/leg.h"
#include <rclcpp/time.hpp>

namespace garlileo {
    using LegFrame = ns_leg::LegFrame;
    class LegDataUnpacker {
    public:
        using Ptr = std::shared_ptr<LegDataUnpacker>;
        
    public:
        explicit LegDataUnpacker() = default;

        static LegDataUnpacker::Ptr Create();

        static LegFrame::Ptr Unpack(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg);

    };

}


#endif 
