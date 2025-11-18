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

#ifndef GARLILEO_BIAS_FILTER_H
#define GARLILEO_BIAS_FILTER_H

#include <ostream>
#include "ctraj/core/spline_bundle.h"
#include <cereal/types/list.hpp>

namespace garlileo {
    class BiasFilter {
    public:
        using Ptr = std::shared_ptr<BiasFilter>;

        struct StatePack {
            double time{};
            Eigen::Vector3d state;
            Eigen::Matrix3d var;

            StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varMat);

            StatePack();

            friend std::ostream &operator<<(std::ostream &os, const StatePack &pack);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(time), CEREAL_NVP(state), CEREAL_NVP(var));
            }
        };

    private:
        StatePack curState;
        const double sigma2;

        std::list<StatePack> stateRecords;

    public:
        BiasFilter(StatePack init, double randomWalk);

        static Ptr Create(const StatePack &init, double randomWalk);

        [[nodiscard]] StatePack Prediction(double t) const;

        [[nodiscard]] const StatePack &GetCurState() const;

        [[nodiscard]] const std::list<StatePack> &GetStateRecords() const;

        void Update(const StatePack &mes);

        void UpdateByEstimator(const StatePack &est);
    };


    class VelBiasFilter {
    public:
        using Ptr = std::shared_ptr<VelBiasFilter>;

        struct StatePack {
            double time{};
            Eigen::Vector2d state;
            Eigen::Matrix2d var;

            StatePack(double time, Eigen::Vector2d state, const Eigen::Vector2d &varMat);

            StatePack();

            friend std::ostream &operator<<(std::ostream &os, const StatePack &pack);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(time), CEREAL_NVP(state), CEREAL_NVP(var));
            }
        };

    private:
        StatePack curState;
        const double sigma2;

        std::list<StatePack> stateRecords;

    public:
        VelBiasFilter(StatePack init, double randomWalk);

        static Ptr Create(const StatePack &init, double randomWalk);

        [[nodiscard]] StatePack Prediction(double t) const;

        [[nodiscard]] const StatePack &GetCurState() const;

        [[nodiscard]] const std::list<StatePack> &GetStateRecords() const;

        void UpdateByEstimator(const StatePack &est);
    };
}


#endif //GARLILEO_BIAS_FILTER_H
