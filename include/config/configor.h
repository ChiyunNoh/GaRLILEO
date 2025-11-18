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

#ifndef GARLILEO_CONFIGOR_H
#define GARLILEO_CONFIGOR_H

#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/set.hpp"
#include "util/utils.hpp"
#include "util/enum_cast.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include "sensor/leg_data_loader.h"
#include "sensor/foot_data_loader.h"

namespace garlileo {
    struct Configor {
    public:
        using Ptr = std::shared_ptr<Configor>;

    public:
        struct DataStream {
            std::string IMUTopic;
            std::string IMUMsgType;

            std::string RadarTopic;
            std::string RadarMsgType;

            std::string LegVelTopic;
            std::string FootTopic;

            CalibParamManager CalibParam;

            std::string OutputPath;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(IMUTopic), CEREAL_NVP(IMUMsgType),
                   CEREAL_NVP(RadarTopic), CEREAL_NVP(LegVelTopic), CEREAL_NVP(FootTopic), CEREAL_NVP(RadarMsgType),
                   CEREAL_NVP(CalibParam), CEREAL_NVP(OutputPath));
            }
        } dataStream;

        struct Prior {
            static constexpr int SplineOrder = 3;

            double GravityNorm;
            double GravityDirection;

            double SO3SplineKnotDist;
            double VelSplineKnotDist;
            double GravSplineKnotDist;

            double AcceWeight;
            double AcceBiasRandomWalk;
            double GyroWeight;
            double GyroBiasRandomWalk;
            double RadarWeight;
            double LegWeight;

            double CauchyLossForRadarFactor;

            double RotGravWeight;
            double GravityWeight;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(GravityNorm), CEREAL_NVP(GravityDirection),
                        CEREAL_NVP(SO3SplineKnotDist), CEREAL_NVP(VelSplineKnotDist), CEREAL_NVP(GravSplineKnotDist),
                        CEREAL_NVP(AcceWeight), CEREAL_NVP(AcceBiasRandomWalk),
                        CEREAL_NVP(GyroWeight), CEREAL_NVP(GyroBiasRandomWalk),
                        CEREAL_NVP(RadarWeight), CEREAL_NVP(LegWeight),
                        CEREAL_NVP(CauchyLossForRadarFactor),
                        CEREAL_NVP(RotGravWeight), CEREAL_NVP(GravityWeight)
                );
            }
        } prior{};

        struct Preference {
            /**
             * when the mode is 'DEBUG_MODE', then:
             * 1. the solving information from ceres would be output on the console
             * 2.
             */
            static bool DEBUG_MODE;

            static std::string SO3Spline;
            static std::string VelSpline;

            // these two splines are not used currently
            static std::string BaSpline;
            static std::string BgSpline;
            
            static std::string GravitySpline;

            static std::string PublishTopic;

            static double StatePublishDelay;

            std::uint32_t IMUMsgQueueSize;
            std::uint32_t RadarMsgQueueSize;
            std::uint32_t LegMsgQueueSize;
            std::uint32_t FootMsgQueueSize;
            std::uint32_t IncrementalOptRate;

            bool OutputResultsWithTimeAligned;


        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(IMUMsgQueueSize), CEREAL_NVP(RadarMsgQueueSize), CEREAL_NVP(LegMsgQueueSize), CEREAL_NVP(FootMsgQueueSize),
                        CEREAL_NVP(IncrementalOptRate), CEREAL_NVP(OutputResultsWithTimeAligned)
                );
            }
        } preference{};

    public:
        Configor();

        static Ptr Create();

        // load configure information from the xml file
        static Configor::Ptr
        Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // load configure information from the xml file
        void Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // print the main fields
        void PrintMainFields();

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("DataStream", dataStream),
                    cereal::make_nvp("Prior", prior),
                    cereal::make_nvp("Preference", preference)
            );
        }
    };
}


#endif //GARLILEO_CONFIGOR_H
