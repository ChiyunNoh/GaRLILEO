// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef GARLILEO_GRAVITY_UPDATE_FACTOR_HPP
#define GARLILEO_GRAVITY_UPDATE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct GravityUpdateFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
    private:
        const double st, et;
        Eigen::Vector3d deltaVel;
        Sophus::SO3d rot1;
        Sophus::SO3d rot2;

        double gravDtInv, velDtInv, so3DtInv, weight;
        std::pair<std::size_t, double> gravIU, vel1IU, vel2IU;
        std::size_t GRAV_OFFSET, BA_OFFSET, VEL1_OFFSET, VEL2_OFFSET;

    public:
        GravityUpdateFactor(const SplineMetaType &gravMeta, const SplineMetaType &velMeta, const double st, const double et, const Eigen::Vector3d &deltaVel,
                             Sophus::SO3d& rot1, Sophus::SO3d& rot2, double weight)
                : st(st), et(et), deltaVel(deltaVel), rot1(rot1), rot2(rot2), gravDtInv(1.0 / gravMeta.segments.front().dt), velDtInv(1.0 / velMeta.segments.front().dt), weight(weight){

                    gravMeta.template ComputeSplineIndex(st, gravIU.first, gravIU.second);
                    velMeta.template ComputeSplineIndex(st, vel1IU.first, vel1IU.second);
                    velMeta.template ComputeSplineIndex(et, vel2IU.first, vel2IU.second);

                    GRAV_OFFSET = gravIU.first;
                    VEL1_OFFSET = gravMeta.NumParameters() + vel1IU.first;
                    VEL2_OFFSET = gravMeta.NumParameters() + vel2IU.first;
                }

            
        static auto
        Create(const SplineMetaType &gravMeta, const SplineMetaType &velMeta, const double st, const double et, const Eigen::Vector3d &deltaVel,
                           Sophus::SO3d& rot1, Sophus::SO3d& rot2, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<GravityUpdateFactor>(
                    new GravityUpdateFactor(gravMeta, velMeta, st, et, deltaVel, rot1, rot2, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(GravityUpdateFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            
            Eigen::Vector3<T> GRAV_IN_CUR;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + GRAV_OFFSET, gravIU.second, gravDtInv, &GRAV_IN_CUR
            );
            
            Eigen::Vector3<T> LIN_VEL_CurToRefInCur1, LIN_VEL_CurToRefInCur2;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL1_OFFSET, vel1IU.second, velDtInv, &LIN_VEL_CurToRefInCur1
            );
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL2_OFFSET, vel2IU.second, velDtInv, &LIN_VEL_CurToRefInCur2
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            // Compute residuals
            residuals = T(weight)*(-GRAV_IN_CUR + (rot1.inverse() * rot2 * LIN_VEL_CurToRefInCur2 - LIN_VEL_CurToRefInCur1  - deltaVel.template cast<T>()) / T(et-st));
            return true;
        }
    };
}
#endif