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

#ifndef GARLILEO_DATA_MANAGER_H
#define GARLILEO_DATA_MANAGER_H

#include "config/configor.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "core/status.h"

#include "sensor/leg_data_loader.h"
#include "sensor/foot_data_loader.h"

#include <spot_msgs/msg/foot_state.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <functional>

using namespace std::placeholders;

namespace garlileo {
    // unique locks for mutexes
#define LOCK_IMU_DATA_SEQ std::unique_lock<std::mutex> imuDataLock(DataManager::IMUDataSeqMutex);
#define LOCK_RADAR_DATA_SEQ std::unique_lock<std::mutex> radarDataLock(DataManager::RadarDataSeqMutex);
#define LOCK_LEG_DATA_SEQ std::unique_lock<std::mutex> legDataLock(DataManager::LegDataSeqMutex);
#define LOCK_FOOT_DATA_SEQ std::unique_lock<std::mutex> footDataLock(DataManager::FootDataSeqMutex);
#define LOCK_RADAR_INIT_FRAMES std::unique_lock<std::mutex> radarInitFramesLock(DataManager::RadarInitFramesMutex);

    class DataManager {
    public:
        using Ptr = std::shared_ptr<DataManager>;

    private:
        rclcpp::Node::SharedPtr handler;
        Configor::Ptr configor;

        // ros subscribers
        rclcpp::SubscriptionBase::SharedPtr imuSuber;
        rclcpp::SubscriptionBase::SharedPtr radarSuber;
        rclcpp::SubscriptionBase::SharedPtr leg_vel_Suber;
        rclcpp::SubscriptionBase::SharedPtr foot_Suber;

        // containers to store sensor data
        std::list<IMUFrame::Ptr> imuDataSeq;
        std::list<RadarTarget::Ptr> radarDataSeq;
        std::list<LegFrame::Ptr> legDataSeq;
        std::list<FootFrame::Ptr> footDataSeq;

        // the radar target array for initialization, size: 10
        std::list<RadarTargetArray::Ptr> radarTarAryForInit;

        std::optional<double> GARLILEO_TIME_EPOCH;

        bool contact=false;
        int contact_duration = 0;

        bool imu_is_come = false;
        Eigen::Quaternion<double> init_orient = Eigen::Quaternion<double>::Identity();
        Eigen::Quaternion<double> prev_orient = Eigen::Quaternion<double>::Identity();
        Eigen::Quaternion<double> SO3_W2Ref = Eigen::Quaternion<double>::Identity();
        

    public:
        // mutexes employed in multi-thread framework
        static std::mutex IMUDataSeqMutex;
        static std::mutex RadarDataSeqMutex;
        static std::mutex LegDataSeqMutex;
        static std::mutex FootDataSeqMutex;
        static std::mutex RadarInitFramesMutex;

    public:
        explicit DataManager(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor);

        static Ptr Create(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor);

        [[nodiscard]] const std::optional<double> &GetGaRLILEOTimeEpoch() const;

        [[nodiscard]] double GetEndTimeSafely() const;

        [[nodiscard]] double GetIMUEndTimeSafely() const;

        [[nodiscard]] double GetRadarEndTimeSafely() const;

        const std::list<IMUFrame::Ptr> &GetIMUDataSeq() const;

        const std::list<RadarTarget::Ptr> &GetRadarDataSeq() const;

        void ShowDataStatus() const;

        [[nodiscard]] std::list<RadarTargetArray::Ptr> GetRadarTarAryForInitSafely() const;

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start, double end);

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start, double end);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start);

        std::list<LegFrame::Ptr> ExtractLegDataPieceSafely(double start, double end);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> AcceMeanVar(const std::list<IMUFrame::Ptr> &data);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> GyroMeanVar(const std::list<IMUFrame::Ptr> &data);

        void EraseOldDataPieceSafely(double time);

    protected:
        IMUFrame::Ptr ApplySMA(const IMUFrame::Ptr& frame) {

            Eigen::Vector3d sumLinearAccel;
            sumLinearAccel.x() = 0.0;
            sumLinearAccel.y() = 0.0;
            sumLinearAccel.z() = 0.0;
            size_t count = 0;

            auto it = imuDataSeq.rbegin();
            for (; it != imuDataSeq.rend() && count < 9; ++it) {
                const auto& prevFrame = *it;
                Eigen::Vector3d temp = prevFrame->GetAcce();
                sumLinearAccel.x() += temp.x();
                sumLinearAccel.y() += temp.y();
                sumLinearAccel.z() += temp.z();
                count++;
            }

            Eigen::Vector3d temp = frame->GetAcce();
            sumLinearAccel.x() += temp.x();
            sumLinearAccel.y() += temp.y();
            sumLinearAccel.z() += temp.z();
            
            count++;

            sumLinearAccel.x() = sumLinearAccel.x() / count;
            sumLinearAccel.y() = sumLinearAccel.y() / count;
            sumLinearAccel.z() = sumLinearAccel.z() / count;
            
            return IMUFrame::Create(frame->GetTimestamp(), frame->GetGyro(), sumLinearAccel, frame->GetOrientation());
        }

        template<class IMUMsgType>
        void HandleIMUMessage(const typename IMUMsgType::ConstSharedPtr &msg) {
            auto frame = IMUDataUnpacker::Unpack(msg);
            if(!imu_is_come){
                imu_is_come = true;
                prev_orient = frame->GetOrientation(); 
                SO3_W2Ref = frame->GetOrientation(); //R_RefW
            }
            else{
                Eigen::Matrix3d R_wo = init_orient.toRotationMatrix();
                
                Eigen::Matrix3d R_wc = frame->GetOrientation().toRotationMatrix();
                Eigen::Matrix3d delta_R = prev_orient.toRotationMatrix().inverse() * R_wc;
                Eigen::Vector3d gyro = frame->GetGyro();
                double dt = frame->GetTimestamp() - imuDataSeq.back()->GetTimestamp() - *GARLILEO_TIME_EPOCH;

                Eigen::AngleAxisd rollAngle(gyro(0)*dt, Eigen::Vector3d::UnitX());   
                Eigen::AngleAxisd pitchAngle(gyro(1)*dt, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(gyro(2)*dt, Eigen::Vector3d::UnitZ()); 

                Eigen::Matrix3d R_no_yaw = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

                Eigen::Matrix3d R_oc = R_wo * R_no_yaw;
                double pitch = atan2(-delta_R(2, 0), sqrt(delta_R(2, 1) * delta_R(2, 1) + delta_R(2, 2) * delta_R(2, 2))); // theta
                double roll = atan2(delta_R(2, 1), delta_R(2, 2));          
                double yaw = atan2(R_oc(1, 0), R_oc(0, 0));           

                Eigen::AngleAxisd rollAngle_angular(roll, Eigen::Vector3d::UnitX());   
                Eigen::AngleAxisd pitchAngle_angular(pitch, Eigen::Vector3d::UnitY()); 
                Eigen::AngleAxisd yawAngle_angular(yaw, Eigen::Vector3d::UnitZ());    

                Eigen::Matrix3d R_refine = (yawAngle_angular * pitchAngle_angular * rollAngle_angular).toRotationMatrix();

                init_orient = Eigen::Quaternion<double>(R_refine);                 
            }

            Eigen::Quaternion<double> relative_orientation = Eigen::Quaternion<double>(init_orient);

            frame->SetOrientation(relative_orientation);
            if (GARLILEO_TIME_EPOCH == std::nullopt) {
                GARLILEO_TIME_EPOCH = frame->GetTimestamp();
                spdlog::info("time epoch of GaRLILEO initialized by imu frame: {:.6f}", *GARLILEO_TIME_EPOCH);
            }
            frame->SetTimestamp(frame->GetTimestamp() - *GARLILEO_TIME_EPOCH);
            {
                LOCK_IMU_DATA_SEQ
                auto smoothedFrame = ApplySMA(frame);
                if(!imuDataSeq.empty()){
                    if(contact_duration>2){
                        imuDataSeq.push_back(smoothedFrame); 
                    }
                    else{
                        auto frame_ = IMUFrame::Create(frame->GetTimestamp(), frame->GetGyro(), imuDataSeq.back()->GetAcce(), frame->GetOrientation());
                        imuDataSeq.push_back(frame_); 
                    }
                }
                else{
                    imuDataSeq.push_back(frame); 
                }
            }
            
        }

        template<class FootMsgType>
        void HandleFootMessage(const typename FootMsgType::ConstSharedPtr &msg) {
            auto frame = FootDataUnpacker::Unpack(msg);
            if (GARLILEO_TIME_EPOCH == std::nullopt) {
                GARLILEO_TIME_EPOCH = frame->GetTimestamp();
                spdlog::info("time epoch of GaRLILEO initialized by imu frame: {:.6f}", *GARLILEO_TIME_EPOCH);
            }
            frame->SetTimestamp(frame->GetTimestamp() - *GARLILEO_TIME_EPOCH);
            {
                LOCK_FOOT_DATA_SEQ
                if(!footDataSeq.empty()){
                    Eigen::Vector4d contact_prev = footDataSeq.back()->GetContactLegs();
                    Eigen::Vector4d contact_curr = frame->GetContactLegs();
                    if(!contact_prev.isApprox(contact_curr)){
                        contact_duration = 0;
                    }
                    if(contact_prev.isApprox(contact_curr)){
                        contact = false;
                        contact_duration ++;
                    }
                }
                footDataSeq.push_back(frame);
            }
        }

        template<class LegMsgType>
        void HandleLegMessage(const typename LegMsgType::ConstSharedPtr &msg) {
            auto frame = LegDataUnpacker::Unpack(msg);
            if (GARLILEO_TIME_EPOCH == std::nullopt) {
                GARLILEO_TIME_EPOCH = frame->GetTimestamp();
                spdlog::info("time epoch of GaRLILEO initialized by Leg frame: {:.6f}", *GARLILEO_TIME_EPOCH);
            }
            frame->SetTimestamp(frame->GetTimestamp() - *GARLILEO_TIME_EPOCH);
            {
                LOCK_LEG_DATA_SEQ
                legDataSeq.push_back(frame);
            }
        }

        template<class RadarMsgType>
        void HandleRadarMessage(const typename RadarMsgType::ConstSharedPtr &msg) {
            auto targetAry = RadarDataUnpacker::Unpack(msg);
            if (GARLILEO_TIME_EPOCH == std::nullopt) {
                GARLILEO_TIME_EPOCH = targetAry->GetTimestamp();
                spdlog::info("time epoch of GaRLILEO initialized by radar targetAry: {:.6f}", *GARLILEO_TIME_EPOCH);
            }
            // aligned the time of targetAry (i.e., radar time) to IMU time
            targetAry->SetTimestamp(
                    targetAry->GetTimestamp() - *GARLILEO_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
            );
            for (auto &tar: targetAry->GetTargets()) {
                tar->SetTimestamp(
                        tar->GetTimestamp() - *GARLILEO_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
                );
            }
            {
                LOCK_RADAR_DATA_SEQ
                radarDataSeq.insert(
                        radarDataSeq.end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend()
                );
            }
            auto status = GaRLILEOStatus::GetStatusPackSafely();
            if (!GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                OrganizeRadarTarAryForInit(targetAry);
            } else {
                {
                    LOCK_RADAR_INIT_FRAMES
                    radarTarAryForInit.clear();
                }
                if (GaRLILEOStatus::IsWith(GaRLILEOStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    LOCK_GARLILEO_STATUS
                    GaRLILEOStatus::DataManager::CurStatus ^= GaRLILEOStatus::DataManager::Status::RadarTarAryForInitIsReady;
                }
            }
        }

        inline void OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry);
    };

}


#endif 
