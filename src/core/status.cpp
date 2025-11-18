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


#include "core/status.h"

namespace garlileo {
    // ------------------------
    // static initialized filed
    // ------------------------
    std::mutex GaRLILEOStatus::StatusMutex = {};

    GaRLILEOStatus::StateManager::Status GaRLILEOStatus::StateManager::CurStatus = GaRLILEOStatus::StateManager::Status::NONE;
    double GaRLILEOStatus::StateManager::ValidStateEndTime = -1.0;

    GaRLILEOStatus::DataManager::Status GaRLILEOStatus::DataManager::CurStatus = GaRLILEOStatus::DataManager::Status::NONE;

    GaRLILEOStatus::StatusPack::StatusPack(GaRLILEOStatus::StateManager::Status stateMagr, double ValidStateEndTime,
                                        GaRLILEOStatus::DataManager::Status dataMagr)
            : StateMagr(stateMagr), ValidStateEndTime(ValidStateEndTime), DataMagr(dataMagr) {}

    GaRLILEOStatus::StatusPack GaRLILEOStatus::GetStatusPackSafely() {
        LOCK_GARLILEO_STATUS
        return {StateManager::CurStatus, StateManager::ValidStateEndTime, DataManager::CurStatus};
    }
}