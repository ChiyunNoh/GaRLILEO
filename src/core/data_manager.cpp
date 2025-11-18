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

#include "core/data_manager.h"

namespace garlileo {
    // -------------------
    // static member field
    // -------------------
    std::mutex DataManager::IMUDataSeqMutex = {};
    std::mutex DataManager::RadarDataSeqMutex = {};
    std::mutex DataManager::LegDataSeqMutex = {};
    std::mutex DataManager::FootDataSeqMutex ={};
    std::mutex DataManager::RadarInitFramesMutex = {};

    DataManager::DataManager(const rclcpp::Node::SharedPtr &handler, const garlileo::Configor::Ptr &configor) 
            : handler(handler), configor(configor), GARLILEO_TIME_EPOCH(std::optional<double>()) {
        spdlog::info("'DataManager' has been booted, thread id: {}.", GARLILEO_TO_STR(std::this_thread::get_id()));
        // -------------------------------------------------------
        // create imu message subscriber based on the message type
        // -------------------------------------------------------

        auto cg_imu   = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cg_radar = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cg_leg   = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cg_foot   = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto make_opt = [](rclcpp::CallbackGroup::SharedPtr cg){
            rclcpp::SubscriptionOptions opt;
            opt.callback_group = cg;
            return opt;
        };



        IMUMsgType imuMsgType;
        try {
            imuMsgType = EnumCast::stringToEnum<IMUMsgType>(configor->dataStream.IMUMsgType); 
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported IMU Type: '{}'. ",
                            configor->dataStream.IMUMsgType
                    )
            );
        }
        /**
         * create subscriber of imu data
         * @topic configor->dataStream.IMUTopic
         * @queue_size configor->preference.IMUMsgQueueSize
         * @callback HandleIMUMessage(...)
         */
           
        switch (imuMsgType) {
            case IMUMsgType::SENSOR_IMU:
                imuSuber = handler->create_subscription<sensor_msgs::msg::Imu>(
                configor->dataStream.IMUTopic,
                rclcpp::QoS(configor->preference.IMUMsgQueueSize),
                std::bind(&DataManager::HandleIMUMessage<sensor_msgs::msg::Imu>, this, _1),
                make_opt(cg_imu));    
                break;
        }
        // ---------------------------------------------------------
        // create radar message subscriber based on the message type
        // ---------------------------------------------------------
        RadarMsgType radarMsgType;
        try {
            radarMsgType = EnumCast::stringToEnum<RadarMsgType>(configor->dataStream.RadarMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            configor->dataStream.RadarMsgType
                    )
            );
        }

        switch (radarMsgType) {
            case RadarMsgType::GARLILEO:
                radarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.RadarTopic,
                rclcpp::QoS(configor->preference.RadarMsgQueueSize),
                std::bind(&DataManager::HandleRadarMessage<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_radar));
                break;
        }
        
        leg_vel_Suber = handler->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        configor->dataStream.LegVelTopic,
        rclcpp::QoS(configor->preference.LegMsgQueueSize),
        std::bind(&DataManager::HandleLegMessage<geometry_msgs::msg::TwistWithCovarianceStamped>, this, _1),
        make_opt(cg_leg));

        foot_Suber = handler->create_subscription<spot_msgs::msg::FootStateArray>(
        configor->dataStream.FootTopic,
        rclcpp::QoS(configor->preference.FootMsgQueueSize),                       
        std::bind(&DataManager::HandleFootMessage<spot_msgs::msg::FootStateArray>, this, _1),
        make_opt(cg_foot));
    }

    DataManager::Ptr DataManager::Create(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor) {
        return std::make_shared<DataManager>(handler, configor);
    }

    void DataManager::ShowDataStatus() const {
        std::size_t s = 0;
        double st = 0.0, et = 0.0;
        {
            // lock imu data and obtain its size, start time, and end time quickly
            LOCK_IMU_DATA_SEQ
            if (!imuDataSeq.empty()) {
                s = imuDataSeq.size();
                st = imuDataSeq.front()->GetTimestamp();
                et = imuDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("imu data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);

        s = 0, st = 0.0, et = 0.0;
        {
            // lock radar data and obtain its size, start time, and end time quickly
            LOCK_RADAR_DATA_SEQ
            if (!radarDataSeq.empty()) {
                s = radarDataSeq.size();
                st = radarDataSeq.front()->GetTimestamp();
                et = radarDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("radar data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);
    }

    void DataManager::OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry) {
        // if this system has not been initialization, construct radar rawTarAry arrays
        static std::vector<RadarTarget::Ptr> tarAry = {};

        tarAry.insert(tarAry.end(), rawTarAry->GetTargets().cbegin(), rawTarAry->GetTargets().cend());

        static double scanHeadTime = tarAry.front()->GetTimestamp();

        if (tarAry.back()->GetTimestamp() - scanHeadTime < 0.1) { return; }

        // if time span is larger than 0.1 (s), try to organize these targets as a radar frame
        if (tarAry.size() < 3) {
            // this radar frame is invalid, clear current status
            tarAry.clear();

            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.clear();
            return;
        }

        // this radar frame is valid, compute average time
        double avgTime = 0.0;
        for (const auto &item: tarAry) { avgTime += item->GetTimestamp(); }
        avgTime /= static_cast<double>(tarAry.size());

        {
            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.push_back(RadarTargetArray::Create(avgTime, tarAry));

            if (radarTarAryForInit.size() > 10) {
                // keep data that lasting for 1.0 (s), i.e., ten frames, each lasting 0.1 (s)
                radarTarAryForInit.pop_front();

                LOCK_GARLILEO_STATUS
                GaRLILEOStatus::DataManager::CurStatus |= GaRLILEOStatus::DataManager::Status::RadarTarAryForInitIsReady;
            }
        }

        // clear current status
        scanHeadTime = tarAry.back()->GetTimestamp();
        tarAry.clear();
    }

    std::list<RadarTargetArray::Ptr> DataManager::GetRadarTarAryForInitSafely() const {
        LOCK_RADAR_INIT_FRAMES
        return radarTarAryForInit;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start, double end) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [end](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        std::copy(sIter, imuDataSeq.end(), std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start, double end) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [end](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;
        
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        std::copy(sIter, radarDataSeq.end(), std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<LegFrame::Ptr> DataManager::ExtractLegDataPieceSafely(double start, double end) {
        LOCK_LEG_DATA_SEQ
        std::list<LegFrame::Ptr> dataSeq; 
        auto sIter = std::find_if(legDataSeq.rbegin(), legDataSeq.rend(), [start](const LegFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(legDataSeq.rbegin(), legDataSeq.rend(), [end](const LegFrame::Ptr &frame) {
            return frame->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(dataSeq));
        return dataSeq;
    }

    double DataManager::GetEndTimeSafely() const {
        double m1, m2;
        {
            LOCK_IMU_DATA_SEQ
            m1 = imuDataSeq.back()->GetTimestamp();
        }
        {
            LOCK_RADAR_DATA_SEQ
            m2 = radarDataSeq.back()->GetTimestamp();
        }
        return std::min(m1, m2);
    }

    const std::optional<double> &DataManager::GetGaRLILEOTimeEpoch() const {
        return GARLILEO_TIME_EPOCH;
    }

    double DataManager::GetIMUEndTimeSafely() const {
        LOCK_IMU_DATA_SEQ
        return imuDataSeq.back()->GetTimestamp();
    }

    double DataManager::GetRadarEndTimeSafely() const {
        LOCK_RADAR_DATA_SEQ
        return radarDataSeq.back()->GetTimestamp();
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::AcceMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetAcce(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::GyroMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetGyro(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    void DataManager::EraseOldDataPieceSafely(double time) {
        {
            LOCK_IMU_DATA_SEQ
            auto iter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [time](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < time;
            }).base();
            imuDataSeq.erase(imuDataSeq.begin(), iter);
        }
        {
            LOCK_RADAR_DATA_SEQ
            auto iter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [time](const RadarTarget::Ptr &tar) {
                return tar->GetTimestamp() < time;
            }).base();
            radarDataSeq.erase(radarDataSeq.begin(), iter);
        }
        {
            LOCK_LEG_DATA_SEQ
            auto iter = std::find_if(legDataSeq.rbegin(), legDataSeq.rend(), [time](const LegFrame::Ptr &frame) {
                return frame->GetTimestamp() < time;
            }).base();
            legDataSeq.erase(legDataSeq.begin(), iter);
        }
    }

    const std::list<IMUFrame::Ptr> &DataManager::GetIMUDataSeq() const {
        return imuDataSeq;
    }

    const std::list<RadarTarget::Ptr> &DataManager::GetRadarDataSeq() const {
        return radarDataSeq;
    }

}

