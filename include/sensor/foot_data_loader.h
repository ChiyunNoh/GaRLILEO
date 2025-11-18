// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef FOOT_DATA_LOADER_H
#define FOOT_DATA_LOADER_H

#include <spot_msgs/msg/foot_state.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include "sensor/foot.h"
#include <rclcpp/rclcpp.hpp>


namespace garlileo {
    using FootFrame = ns_foot::FootFrame;
    class FootDataUnpacker {
    public:
        using Ptr = std::shared_ptr<FootDataUnpacker>;
        
    public:
        explicit FootDataUnpacker() = default;

        static FootDataUnpacker::Ptr Create();

        static FootFrame::Ptr Unpack(const spot_msgs::msg::FootStateArray::ConstSharedPtr& msg);

    };

}


#endif 
