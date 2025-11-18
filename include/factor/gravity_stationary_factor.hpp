// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef GRAVITY_STATIONARY_FACTOR_HPP
#define GRAVITY_STATIONARY_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    struct GravityStationaryFactor {
    private:
        Eigen::Vector3d acceMean;
        double weight;

    public:
        GravityStationaryFactor(Eigen::Vector3d &acceMean, double weight)
                : acceMean(std::move(acceMean)), weight(weight) {}

        static auto
        Create(Eigen::Vector3d &acceMean, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<GravityStationaryFactor>(
                    new GravityStationaryFactor(acceMean, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(GravityStationaryFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = acceMean + gravity;
            residuals = T(weight) * residuals;

            return true;
        }
    };
}
#endif 
