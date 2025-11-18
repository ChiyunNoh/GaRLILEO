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

#ifndef GARLILEO_H
#define GARLILEO_H
#include "config/configor.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "core/state_manager.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace garlileo {
    class GaRLILEO {
    public:
        using Ptr = std::shared_ptr<GaRLILEO>;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr handler;
        Configor::Ptr           configor;

        DataManager::Ptr                 dataMagr;
        StateManager::Ptr                stateMagr;
        std::shared_ptr<std::thread>     stateMagrThread;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     posePublisher;

        nav_msgs::msg::Path pose_path;
        nav_msgs::msg::Path pose_path_G;

        bool first_publish_flag    = true;
        bool first_publish_flag_G  = true;

        double prev_time                 = 0.0;
        double last_timestamp_pose_pub_  = 0.0;
        double prev_time_G               = 0.0;
        double last_timestamp_pose_pub_G = 0.0;
        double trajectory_length         = 0.0;

        // std::string rpm = R"(
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⢤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡍⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣷⡙⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣴⡿⠛⢿⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣺⡟⠃⠀⠀⠀⠙⢿⣧⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣽⣧⠀⢿⠂⠀⢿⠇⢀⣽⢿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠟⣥⣮⡹⢿⣦⡀⠀⢀⣴⡿⢋⣵⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣔⣮⣧⡿⠏⠛⣿⣦⡙⢿⣶⡿⢋⣴⡿⠋⢻⣷⡷⣿⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠈⣹⣿⠄⠀⠀⠙⢿⣦⣩⣶⠿⠋⠀⠀⠺⣿⣄⠀⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀
        // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠁⠀⠀⠀⠀⠀⠙⠿⠉⠀⠀⠀⠀⠀⠈⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀                                                                
        //               ___  ___  __  ___
        //              / _ \/ _ \/  |/  /
        //             / , _/ ___/ /|_/ / 
        //            /_/|_/_/  /_/  /_/  
        //     ___  ____  ___  ____  ___________________
        //    / _ \/ __ \/ _ )/ __ \/_  __/  _/ ___/ __/
        //   / , _/ /_/ / _  / /_/ / / / _/ // /___\ \  
        //  /_/|_|\____/____/\____/ /_/ /___/\___/___/  
        // )"; 
        std::string rpm = R"(                               
         ______      ____  __    ______    __________ 
        / ____/___ _/ __ \/ /   /  _/ /   / ____/ __ \
       / / __/ __ `/ /_/ / /    / // /   / __/ / / / /
      / /_/ / /_/ / _, _/ /____/ // /___/ /___/ /_/ / 
      \____/\__,_/_/ |_/_____/___/_____/_____/\____/   
        )"; 

    public:
        explicit GaRLILEO(const Configor::Ptr &configor);

        static Ptr Create(const Configor::Ptr &configor);

        void Run();

        void Save();

        void save_pose_tum(const std::string& filename, 
                   const std::vector<std::pair<double, Eigen::Vector3d>>& velocity,
                   const std::vector<std::pair<double, Sophus::SO3d>>& quatVec);
        void log_fancy(double current_time_s, geometry_msgs::msg::PoseStamped& pose_stamped, std::optional<StateManager::StatePack> &status);
        inline std::string string_from_double(double d, int precision = 3) {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(precision) << d;
            return ss.str();
        }   

    protected:
        void PublishGaRLILEOState(const GaRLILEOStatus::StatusPack &status);
    };
}

#endif 