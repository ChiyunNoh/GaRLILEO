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

#include "core/garlileo.h"
#include "cereal/types/utility.hpp"
using std::placeholders::_1;

namespace garlileo {

    GaRLILEO::GaRLILEO(const Configor::Ptr &configor)
            : handler(rclcpp::Node::make_shared("garlileo")), configor(configor),
              dataMagr(DataManager::Create(handler, configor)),
              stateMagr(StateManager::Create(dataMagr, configor)),
              stateMagrThread(std::make_shared<std::thread>(&StateManager::Run, stateMagr)) {

        posePublisher     = handler->create_publisher<nav_msgs::msg::Path>(
                                "/path",
                                rclcpp::QoS(100'000));

        odomPublisher     = handler->create_publisher<nav_msgs::msg::Odometry>(
                                "/Odometry",
                                rclcpp::QoS(100'000));

    }

    GaRLILEO::Ptr GaRLILEO::Create(const Configor::Ptr &configor) {
        return std::make_shared<GaRLILEO>(configor);
    }

    void GaRLILEO::Run()
    {
        auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        exec->add_node(handler);
        std::thread spin_thr([&](){ exec->spin(); });

        rclcpp::Rate rate(configor->preference.IncrementalOptRate);

        while (rclcpp::ok()) {
            auto status = GaRLILEOStatus::GetStatusPackSafely();
            if (GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("GaRLILEO StateManager thread quits.");
                break;
            }

            PublishGaRLILEOState(status);

            if (GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                dataMagr->EraseOldDataPieceSafely(status.ValidStateEndTime - 0.2);
            }
            rate.sleep();
        }
        exec->cancel();
        spin_thr.join();
        stateMagrThread->join();
    }

        void GaRLILEO::save_pose_tum(const std::string& filename, 
                   const std::vector<std::pair<double, Eigen::Vector3d>>& velocity,
                   const std::vector<std::pair<double, Sophus::SO3d>>& quatVec) 
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        assert(velocity.size() == quatVec.size());
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(9);
        double t_prev = velocity[0].first;
        file << t_prev << " " << position.x() << " " << position.y() << " " << position.z() << " ";
        auto q0 = quatVec[0].second.unit_quaternion();
        file << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w() << "\n";

        for (size_t i = 1; i < velocity.size(); ++i) {
            double t = velocity[i].first;
            double dt = t - t_prev;
            t_prev = t;

            const Eigen::Vector3d& vel_body = stateMagr->GetSO3_RefToW().matrix().transpose() * velocity[i].second;
            const Sophus::SO3d& so3 = quatVec[i].second; //R_wb
            Eigen::Matrix3d R_wb = stateMagr->GetSO3_RefToW().matrix().transpose() * so3.matrix() * stateMagr->GetSO3_RefToW().matrix();  // body to First IMU(R_I0w * R_wb)

            Eigen::Vector3d vel_world = R_wb * vel_body;
            position += vel_world * dt;

            Sophus::SO3d so3_wb(R_wb);
            Eigen::Quaterniond q_wb = so3_wb.unit_quaternion();
            file << t << " "
                << position.x() << " " << position.y() << " " << position.z() << " "
                << q_wb.x() << " " << q_wb.y() << " " << q_wb.z() << " " << q_wb.w() << "\n";
        }
        file.close();
        
    }



    void GaRLILEO::Save() {
        const std::string &tarDir = configor->dataStream.OutputPath + "/garlileo_output";
        if (!std::filesystem::exists(tarDir)) {
            if (!std::filesystem::create_directories(tarDir)) {
                throw Status(Status::Flag::WARNING, fmt::format(
                        "the output path for data, i.e., '{}', dose not exist and create failed!", tarDir)
                );
            }
        } else {
            std::filesystem::remove_all(tarDir);
            std::filesystem::create_directories(tarDir);
        }

        auto &splines = this->stateMagr->GetSplines();
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        const double epoch = configor->preference.OutputResultsWithTimeAligned ? 0.0 : *dataMagr->GetGaRLILEOTimeEpoch();
        const double velST = velSpline.MinTime(), velET = velSpline.MaxTime();
        const double so3ST = so3Spline.MinTime(), so3ET = so3Spline.MaxTime();

        velSpline.SetStartTime(velST + epoch);
        so3Spline.SetStartTime(so3ST + epoch);

        velSpline.SetStartTime(velST);
        so3Spline.SetStartTime(so3ST);

        // velocity
        const double st = std::max(velST, so3ST), et = std::min(velET, so3ET);

        std::vector<std::pair<double, Eigen::Vector3d>> gravityInB, velocityInB;
        std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
        for (double t = st; t < et;) {
            Eigen::Vector3d LIN_VEL_BtoWinB = velSpline.Evaluate(t);
            velocityInB.emplace_back(t + epoch, LIN_VEL_BtoWinB);

            auto SO3_CurToW = so3Spline.Evaluate(t);
            quatBtoW.emplace_back(t + epoch, SO3_CurToW);

            Eigen::Vector3d GRAV = gravSpline.Evaluate(t);
            gravityInB.emplace_back(t + epoch, GRAV);

            t += 0.01;
        }
        save_pose_tum(tarDir +"/poses.txt", velocityInB, quatBtoW);
        
        spdlog::info("pose of 'GaRLILEO' have been saved to dir '{}'.", tarDir);
    }

    void GaRLILEO::log_fancy(double current_time_s, geometry_msgs::msg::PoseStamped& pose_stamped, std::optional<StateManager::StatePack> &status) {

        std::cout<<"\033[2J\033[1;1H"; //clear screen
        std::cout<<"\033[0m" << rpm <<std::endl; 
        std::cout<<"\033[0m"; 

        std::time_t current_time = std::time(nullptr);
        double elapsed_time = current_time_s;

        std::string asc_time = std::asctime(std::localtime(&current_time)); asc_time.pop_back();
        std::cout << "| " << std::left << asc_time;
        std::cout << std::right << std::setfill(' ') << std::setw(35)
        << "Elapsed Time: " + string_from_double(elapsed_time) + " seconds "
        << "|" << std::endl;

        std::cout << "|------------------------------------------------------------|" << std::endl;

        const auto& p = pose_stamped.pose.position;
        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
                << "Position (x,y,z)    [m] : " 
                + string_from_double(p.x) + " " 
                + string_from_double(p.y) + " " 
                + string_from_double(p.z)
                << "|" << std::endl;

        const auto& o = pose_stamped.pose.orientation;
        const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);

        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); 
        euler *= 180.0 / M_PI;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(60)
                << "Orientation (r,p,y) [°] : "
                + string_from_double(euler(0)) + " "
                + string_from_double(euler(1)) + " "
                + string_from_double(euler(2))
                << "|" << std::endl;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        << "Lin Velocity {B}  [xyz] : " + string_from_double(status->LIN_VEL_CurToRefInCur(0)) + " "
                                    + string_from_double(status->LIN_VEL_CurToRefInCur(1)) + " "
                                    + string_from_double(status->LIN_VEL_CurToRefInCur(2)) << "|" << std::endl;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        << "Trajectory Length   [m] : " + string_from_double(trajectory_length) << "|" << std::endl;

        std::cout << "|------------------------------------------------------------|" << std::endl;


        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        // << "Acc. Bias (x,y,z) [m/s2]  : " + string_from_double(state_point.ba(0)) + " " 
        // + string_from_double(state_point.ba(1)) + " " + string_from_double(state_point.ba(2)) << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        // << "Gyro Bias (x,y,z) [rad/s] : " + string_from_double(state_point.bg(0)) + " " 
        // + string_from_double(state_point.bg(1)) + " " + string_from_double(state_point.bg(2)) << "|" << std::endl;
    
        // std::cout << "|------------------------------------------------------------|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Effective Points    [#] : " <<  std::left << std::setw(33) << n_effective_points << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Intensity Features  [#] : " << std::left << std::setw(6) << n_features << std::left << std::setw(13) 
        // << " Added: " + std::to_string(n_added) << std::left << std::setw(14)<< " Removed: " + std::to_string(n_removed) 
        // << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Uninformative Dir.  [#] : " << std::left << std::setw(33) << n_uninformative << "|" << std::endl;

        // std::cout << "|------------------------------------------------------------|" << std::endl;

        // double mean_s = timing::Timing::GetMeanSeconds("all");
        // double min_s = timing::Timing::GetMinSeconds("all");
        // double max_s = timing::Timing::GetMaxSeconds("all");
        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Computation Time    [s] : " << std::left << "Avg: " << string_from_double(mean_s) << std::left 
        // << " Max: " << string_from_double(max_s)  << std::left << " Min: " << string_from_double(min_s) 
        // << " |" << std::endl;
    }

    void GaRLILEO::PublishGaRLILEOState(const GaRLILEOStatus::StatusPack &status) {
        if (!GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::NewStateNeedToPublish, status.StateMagr)) {
            return;
        }

        std::optional<StateManager::StatePack> state = this->stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - 0.3
        );
        if (state == std::nullopt) { return; }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp    = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
        pose_stamped.header.frame_id = "world";
               
        
        double dt = this->first_publish_flag ? 0.0 : state->timestamp - this->prev_time;
        geometry_msgs::msg::Pose last_pose;
        last_pose = this->first_publish_flag ? geometry_msgs::msg::Pose() : this->pose_path.poses.back().pose;
        this->first_publish_flag = false;
        this->prev_time = state->timestamp;

        pose_stamped.pose.position.x = last_pose.position.x + (state->SO3_CurToRef * state->LIN_VEL_CurToRefInCur * dt)(0);
        pose_stamped.pose.position.y = last_pose.position.y + (state->SO3_CurToRef * state->LIN_VEL_CurToRefInCur * dt)(1);
        pose_stamped.pose.position.z = last_pose.position.z + (state->SO3_CurToRef * state->LIN_VEL_CurToRefInCur * dt)(2);
        pose_stamped.pose.orientation.x = state->SO3_CurToRef.unit_quaternion().x();
        pose_stamped.pose.orientation.y = state->SO3_CurToRef.unit_quaternion().y();
        pose_stamped.pose.orientation.z = state->SO3_CurToRef.unit_quaternion().z();
        pose_stamped.pose.orientation.w = state->SO3_CurToRef.unit_quaternion().w();

        this->pose_path.poses.push_back(pose_stamped);

        // Dashboard logging
        Eigen::Vector3d d_ref = state->SO3_CurToRef * state->LIN_VEL_CurToRefInCur * dt;
        double segment_length = d_ref.norm(); 
        this->trajectory_length += segment_length;

        log_fancy(status.ValidStateEndTime - 0.3, pose_stamped, state);

        if ((state->timestamp - this->last_timestamp_pose_pub_) > 1.0 / 5)
        {
            this->pose_path.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
            this->pose_path.header.frame_id = "world";
            posePublisher->publish(this->pose_path);
            this->last_timestamp_pose_pub_ = state->timestamp;
        }

        nav_msgs::msg::Odometry axis_path;
        axis_path.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
        axis_path.header.frame_id = "world";
        axis_path.pose.pose.position.x = pose_stamped.pose.position.x;
        axis_path.pose.pose.position.y = pose_stamped.pose.position.y;
        axis_path.pose.pose.position.z = pose_stamped.pose.position.z;
        axis_path.pose.pose.orientation.x = state->SO3_CurToRef.unit_quaternion().x();
        axis_path.pose.pose.orientation.y = state->SO3_CurToRef.unit_quaternion().y();
        axis_path.pose.pose.orientation.z = state->SO3_CurToRef.unit_quaternion().z();
        axis_path.pose.pose.orientation.w = state->SO3_CurToRef.unit_quaternion().w();

        odomPublisher->publish(axis_path);


        {
            LOCK_GARLILEO_STATUS
            GaRLILEOStatus::StateManager::CurStatus ^= GaRLILEOStatus::StateManager::Status::NewStateNeedToPublish;
        }
    }
}