// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef GARLILEO_GRAV_SPLINE_FACTOR_HPP
#define GARLILEO_GRAV_SPLINE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct GravitySplineFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double gravDtInv, so3DtInv;

        // array offset
        std::pair<std::size_t, double> gravIU, so3IU;
        std::size_t GRAV_OFFSET, SO3_OFFSET;

    public:
        GravitySplineFactor(const SplineMetaType &gravMeta, const SplineMetaType &so3Meta, IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), gravDtInv(1.0 / gravMeta.segments.front().dt), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            // compute knots indexes
            gravMeta.template ComputeSplineIndex(frame->GetTimestamp(), gravIU.first, gravIU.second);
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);

            // compute knots offset in 'parBlocks'
            GRAV_OFFSET = gravIU.first;
            SO3_OFFSET = gravMeta.NumParameters() + so3IU.first;
        }

        static auto
        Create(const SplineMetaType &gravMeta, const SplineMetaType &so3Meta, const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<GravitySplineFactor>(
                    new GravitySplineFactor(gravMeta, so3Meta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(GravitySplineFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Vector3<T> GRAV_DIFF;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + GRAV_OFFSET, gravIU.second, gravDtInv, &GRAV_DIFF
            );

            Eigen::Vector3<T> GRAV;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + GRAV_OFFSET, gravIU.second, gravDtInv, &GRAV
            );

            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = GRAV_DIFF + Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur) * GRAV;
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif 