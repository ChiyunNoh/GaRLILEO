// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef GARLILEO_ROT_PRIOR_FACTOR_HPP
#define GARLILEO_ROT_PRIOR_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct RotPriorFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
    private:
        double so3DtInv;

        // array offset
        std::pair<std::size_t, double> so3IU1, so3IU2;
        std::size_t SO3_OFFSET1, SO3_OFFSET2;

        double weight, dt;

    public:
        RotPriorFactor(double stime, double dt, const SplineMetaType &so3Meta, double weight)
                : so3DtInv(1.0 / so3Meta.segments.front().dt), weight(weight), dt(dt){
                // compute knots indexes
                so3Meta.template ComputeSplineIndex(stime, so3IU1.first, so3IU1.second);
                so3Meta.template ComputeSplineIndex(stime+dt, so3IU2.first, so3IU2.second);

                SO3_OFFSET1 = so3IU1.first;
                SO3_OFFSET2 = so3IU2.first;
            }


        static auto
        Create(double stime, double dt, const SplineMetaType &so3Meta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RotPriorFactor>(
                    new RotPriorFactor(stime, dt, so3Meta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RotPriorFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3<T> SO3_CurToRef1, SO3_CurToRef2;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET1, so3IU1.second, so3DtInv, &SO3_CurToRef1
            );
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET2, so3IU2.second, so3DtInv, &SO3_CurToRef2
            );

            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur1;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET1, so3IU1.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur1
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = T(dt) * ANG_VEL_CurToRefInCur1 - (SO3_CurToRef1.inverse() * SO3_CurToRef2).log();
            residuals = T(weight) * residuals;

            return true;
        }
    };
}
#endif 
