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


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "thread"
#include "core/status.h"
#include "core/garlileo.h"
 

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    try {
        ConfigSpdlog();

        // obtain the path of config file
        auto node = rclcpp::Node::make_shared("garlileo_node");

        node->declare_parameter<std::string>("config_path", "");
        std::string configPath = node->get_parameter("config_path").as_string();

        if (configPath.empty()) {
            RCLCPP_FATAL(node->get_logger(),
                        "Parameter '/garlileo_node/config_path' not set -- "
                        "run with:  ros2 run river river_prog "
                        " --ros-args -p config_path:=/absolute/path/to/river.yaml");
            rclcpp::shutdown();
            return 1;
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        // load the configure file
        auto configor = garlileo::Configor::Load(configPath); 
        configor->PrintMainFields();

        auto garlileo = garlileo::GaRLILEO::Create(configor);

        garlileo->Run();

        garlileo->Save();

    } catch (const garlileo::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case garlileo::Status::Flag::FINE:
                // this case usually won't happen
                spdlog::info(status.what);
                break;
            case garlileo::Status::Flag::WARNING:
                spdlog::warn(status.what);
                break;
            case garlileo::Status::Flag::ERROR:
                spdlog::error(status.what);
                break;
            case garlileo::Status::Flag::CRITICAL:
                spdlog::critical(status.what);
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }
    rclcpp::shutdown();
    return 0;
}