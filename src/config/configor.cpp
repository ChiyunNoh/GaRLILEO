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

#include "config/configor.h"

namespace garlileo {
    // -------------------
    // static member field
    // -------------------
    std::string Configor::Preference::SO3Spline = "so3";
    std::string Configor::Preference::VelSpline = "vel";
    std::string Configor::Preference::BaSpline = "ba";
    std::string Configor::Preference::BgSpline = "bg";
    std::string Configor::Preference::GravitySpline = "g";

    std::string Configor::Preference::PublishTopic = "/state";
    double Configor::Preference::StatePublishDelay = 0.05;

    bool Configor::Preference::DEBUG_MODE = false;

    Configor::Configor() = default;

    Configor::Ptr Configor::Create() {
        return std::make_shared<Configor>();
    }

    Configor::Ptr Configor::Load(const std::string &filename, CerealArchiveType::Enum archiveType) {
        auto configor = Configor::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *configor));
        return configor;
    }

    void Configor::Save(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *this));
    }

    void Configor::PrintMainFields() {
#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>35}: {}"
        spdlog::info(
        "main fields of configor:"
        // dataStream (5)
        DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
        // prior (10)
        DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
        DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
        // preference (3)
        DESC_FORMAT DESC_FORMAT DESC_FORMAT,
        // --------------------- args ---------------------
        DESC_FIELD(dataStream.IMUTopic),
        DESC_FIELD(dataStream.IMUMsgType),
        DESC_FIELD(dataStream.RadarTopic),
        DESC_FIELD(dataStream.RadarMsgType),
        DESC_FIELD(dataStream.OutputPath),

        DESC_FIELD(prior.SplineOrder),
        DESC_FIELD(prior.GravityNorm),
        DESC_FIELD(prior.SO3SplineKnotDist),
        DESC_FIELD(prior.VelSplineKnotDist),
        DESC_FIELD(prior.AcceWeight),
        DESC_FIELD(prior.AcceBiasRandomWalk),
        DESC_FIELD(prior.GyroWeight),
        DESC_FIELD(prior.GyroBiasRandomWalk),
        DESC_FIELD(prior.RadarWeight),
        DESC_FIELD(prior.CauchyLossForRadarFactor),

        DESC_FIELD(preference.IMUMsgQueueSize),
        DESC_FIELD(preference.RadarMsgQueueSize),
        DESC_FIELD(preference.IncrementalOptRate)
        );
#undef DESC_FIELD
#undef DESC_FORMAT
    }
}// namespace garlileo
