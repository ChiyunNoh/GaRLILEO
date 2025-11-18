// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef LEG_VEL_FACTOR_HPP
#define LEG_VEL_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/leg_data_loader.h"

namespace garlileo {
    template<int Order>
    struct LegVelFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        LegFrame::Ptr frame;
        Sophus::SO3d& SO3_RefToW;

        double weight;
        double so3DtInv, velDtInv;

        std::pair<std::size_t, double> so3IU, velIU;

        std::size_t SO3_OFFSET, VEL_OFFSET, BV_OFFSET;
    public:
        LegVelFactor(const CalibParamManager &calibParMagr, const SplineMetaType &so3Meta, const SplineMetaType &velMeta, LegFrame::Ptr legFrame, Sophus::SO3d& SO3_RefToW, double weight)
                : parMagr(calibParMagr), frame(std::move(legFrame)), SO3_RefToW(SO3_RefToW), weight(weight),  
                so3DtInv(1.0 / so3Meta.segments.front().dt), velDtInv(1.0 / velMeta.segments.front().dt){
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
            BV_OFFSET = so3Meta.NumParameters() +  velMeta.NumParameters();
        }

        static auto Create(const CalibParamManager &calibParMagr, const SplineMetaType &so3Meta, const SplineMetaType &velMeta, LegFrame::Ptr legFrame, Sophus::SO3d& SO3_RefToW, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<LegVelFactor>(
                    new LegVelFactor(calibParMagr, so3Meta, velMeta, legFrame, SO3_RefToW, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(LegVelFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            
            Eigen::Vector3<T> LIN_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_CurToRefInCur
            );

            Sophus::SO3<T> SO3_CurToRef;
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef, &ANG_VEL_CurToRefInCur
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            Eigen::Map<const Eigen::Vector2<T>> BV(parBlocks[BV_OFFSET]);
            residuals = LIN_VEL_CurToRefInCur - 
            (parMagr.SO3_BtoL.matrix().transpose() * frame->GetVel() - Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur ) * (-parMagr.SO3_BtoL.matrix().transpose() * parMagr.POS_BinL));
            residuals(0) -= BV(0);
            residuals(1) -= BV(1);

            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif 