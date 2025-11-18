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

#ifndef GARLILEO_IMU_ACCE_FACTOR_HPP
#define GARLILEO_IMU_ACCE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace garlileo {
    template<int Order>
    struct IMUAcceFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, velDtInv, baDtInv;

        std::pair<std::size_t, double> so3IU, velIU, baIU;

        std::size_t SO3_OFFSET, VEL_OFFSET, BA_OFFSET, GRAVITY_OFFSET;
    public:
        IMUAcceFactor(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const SplineMetaType &baMeta,
                      IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt),
                  velDtInv(1.0 / velMeta.segments.front().dt), baDtInv(1.0 / baMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);
            baMeta.template ComputeSplineIndex(frame->GetTimestamp(), baIU.first, baIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
            BA_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters() + baIU.first;
            GRAVITY_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters() + baMeta.NumParameters();
        }

        static auto Create(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const SplineMetaType &baMeta,
                           const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactor>(
                    new IMUAcceFactor(so3Meta, velMeta, baMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactor).hash_code();
        }

    public:

        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            Eigen::Vector3<T> LIN_ACCE_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_ACCE_CurToRefInRef
            );

            // compute the bias of accelerator
            Eigen::Vector3<T> BA;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + BA_OFFSET, baIU.second, baDtInv, &BA
            );

            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef.inverse() * (LIN_ACCE_CurToRefInRef - GRAVITY_IN_REF) + BA
                        - frame->GetAcce().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct IMUAcceFactorWithConstBias {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, velDtInv;

        std::pair<std::size_t, double> so3IU, velIU;

        std::size_t SO3_OFFSET, VEL_OFFSET, BA_OFFSET, GRAVITY_OFFSET;

    public:
        IMUAcceFactorWithConstBias(const CalibParamManager &calibParMagr, const SplineMetaType &so3Meta, const SplineMetaType &velMeta,
                                   IMUFrame::Ptr imuFrame, double weight)
                : parMagr(calibParMagr), frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt),
                  velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            // ComputeSplineIndex(const T &timestamp, size_t &idx, T &u)
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
            BA_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters();
            GRAVITY_OFFSET = BA_OFFSET + 1;
        }

        static auto Create(const CalibParamManager &calibParMagr, const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const IMUFrame::Ptr &frame,
                           double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactorWithConstBias>(
                    new IMUAcceFactorWithConstBias(calibParMagr, so3Meta, velMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactorWithConstBias).hash_code();
        }

    public:

        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur
            );

            Eigen::Vector3<T> LIN_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_CurToRefInCur
            );

            Eigen::Vector3<T> LIN_ACCE_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_ACCE_CurToRefInCur
            );

            LIN_ACCE_CurToRefInCur += Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur) * LIN_VEL_CurToRefInCur;

            Eigen::Map<const Eigen::Vector3<T>> BA(parBlocks[BA_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            residuals = (LIN_ACCE_CurToRefInCur - SO3_CurToRef.inverse() * GRAVITY_IN_REF) + BA
                        - frame->GetAcce().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif 
