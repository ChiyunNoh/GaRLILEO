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

#ifndef GARLILEO_BIAS_FACTOR_HPP
#define GARLILEO_BIAS_FACTOR_HPP

namespace garlileo {

    struct BiasFactor {
    private:
        Eigen::Vector3d biasPriori;
        Eigen::Matrix3d weight;

    public:
        BiasFactor(Eigen::Vector3d biasPriori, const Eigen::Matrix3d &var) : biasPriori(std::move(biasPriori)) {
            weight = Eigen::LLT<Eigen::Matrix3d>(var.inverse()).matrixL().transpose();
        }

        static auto Create(const Eigen::Vector3d &biasPriori, const Eigen::Matrix3d &var) {
            return new ceres::DynamicAutoDiffCostFunction<BiasFactor>(new BiasFactor(biasPriori, var));
        }

        static std::size_t TypeHashCode() {
            return typeid(BiasFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> bias(parBlocks[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = bias - biasPriori;
            residuals = weight * residuals;

            return true;
        }
    };
}

#endif 
