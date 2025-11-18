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

#ifndef GARLILEO_STATUS_H
#define GARLILEO_STATUS_H

#include "exception"
#include "string"
#include "util/enum_cast.hpp"
#include "mutex"

namespace garlileo {
    struct Status : std::exception {
        enum class Flag {
            FINE, WARNING, ERROR, CRITICAL
        };
    public:
        Flag flag;
        std::string what;

        Status(Flag flag, std::string what) : flag(flag), what(std::move(what)) {}

        Status() : flag(Flag::FINE), what() {}
    };

    using namespace magic_enum::bitwise_operators;

#define LOCK_GARLILEO_STATUS std::unique_lock<std::mutex> statusLock(GaRLILEOStatus::StatusMutex);

    class GaRLILEOStatus {
    public:
        struct StateManager {
            enum class Status : std::uint32_t {
                /**
                 * @brief options
                 */
                NONE = 1 << 0,
                HasInitialized = 1 << 1,
                ShouldQuit = 1 << 2,
                NewStateNeedToDraw = 1 << 3,
                NewStateNeedToPublish = 1 << 4,
            };

            static Status CurStatus;

            static double ValidStateEndTime;
        };

        struct DataManager {
            enum class Status : std::uint32_t {
                /**
                 * @brief options
                 */
                NONE = 1 << 0,
                RadarTarAryForInitIsReady = 1 << 1,
            };

            static Status CurStatus;
        };

        struct StatusPack {
        public:
            StateManager::Status StateMagr;
            double ValidStateEndTime;

            DataManager::Status DataMagr;

        public:
            StatusPack(StateManager::Status stateMagr, double ValidStateEndTime, DataManager::Status dataMagr);
        };

    public:
        static std::mutex StatusMutex;

    public:
        template<class EnumType>
        static bool IsWith(EnumType desired, EnumType current) {
            return (desired == (desired & current));
        }

        static StatusPack GetStatusPackSafely();

    };
}

#endif
