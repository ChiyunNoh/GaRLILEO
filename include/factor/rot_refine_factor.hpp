// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef GARLILEO_ROT_REFINE_FACTOR_HPP
#define GARLILEO_ROT_REFINE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct RotRefineFactor { 
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
    private:

        double weight;

    public:
        RotRefineFactor(double weight)
                : weight(weight){

            }


        static auto
        Create(double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RotRefineFactor>(
                    new RotRefineFactor(weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RotRefineFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<Sophus::SO3<T> const> const SO3_CurToRef(parBlocks[0]);
            Eigen::Map<const Eigen::Vector3<T>> GRAV_IN_CUR(parBlocks[1]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[2]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef * GRAV_IN_CUR - GRAVITY_IN_REF;
            residuals = T(weight) * residuals;

            return true;
        }
    };
}
#endif
