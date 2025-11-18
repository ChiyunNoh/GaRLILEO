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

#ifndef GARLILEO_RADAR_FACTOR_HPP
#define GARLILEO_RADAR_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct RadarFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        RadarTarget::Ptr target;
        Sophus::SO3d SO3_RefToW;

        double weight;
        double so3DtInv, velDtInv;

        // compute knots indexes
        std::pair<std::size_t, double> so3IU, velIU;
        std::size_t SO3_OFFSET, VEL_OFFSET;

    public:
        RadarFactor(const CalibParamManager &calibParMagr, Sophus::SO3d& SO3_RefToW, RadarTarget::Ptr radarTar, const SplineMetaType &so3Meta,
                    const SplineMetaType &velMeta, double weight)
                : parMagr(calibParMagr), target(std::move(radarTar)), SO3_RefToW(SO3_RefToW), weight(weight),
                  so3DtInv(1.0 / so3Meta.segments.front().dt), velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(target->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(target->GetTimestamp(), velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
        }

        static auto Create(const CalibParamManager &calibParMagr, Sophus::SO3d& SO3_RefToW, const RadarTarget::Ptr &radarTar,
                           const SplineMetaType &so3Meta, const SplineMetaType &velMeta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RadarFactor>(
                    new RadarFactor(calibParMagr, SO3_RefToW, radarTar, so3Meta, velMeta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RadarFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3<T> SO3_CurToRef;
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef, &ANG_VEL_CurToRefInCur
            );

            Eigen::Vector3<T> LIN_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_CurToRefInCur 
            );

            Eigen::Vector3<T> LIN_VEL_RtoRefInCur =
                    Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur ) * parMagr.POS_RinB +
                    LIN_VEL_CurToRefInCur; //V^w_wr

            T v1 = -target->GetTargetXYZ().cast<T>().dot(
                    parMagr.SO3_RtoB.matrix().transpose() * LIN_VEL_RtoRefInCur
            );

            T v2 = static_cast<T>(target->GetRadialVelocity());

            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
            residuals(0, 0) = T(weight) * (target->GetInvRange() * v1 - v2);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}
#endif 
