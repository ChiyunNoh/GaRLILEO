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

#ifndef GARLILEO_TAIL_FACTOR_HPP
#define GARLILEO_TAIL_FACTOR_HPP

namespace garlileo {
    struct RdTailFactor {
    private:
        double weight;

    public:
        explicit RdTailFactor(double weight) : weight(weight) {}

        static auto Create(double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RdTailFactor>(new RdTailFactor(weight));
        }

        static std::size_t TypeHashCode() {
            return typeid(RdTailFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | VEL | VEL ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> firKnot(parBlocks[0]);
            Eigen::Map<const Eigen::Vector3<T>> sedKnot(parBlocks[1]);
            Eigen::Map<const Eigen::Vector3<T>> thdKnot(parBlocks[2]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = weight * (firKnot - 2.0 * sedKnot + thdKnot);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct So3TailFactor {
    private:
        double weight;

    public:
        explicit So3TailFactor(double weight) : weight(weight) {}

        static auto Create(double weight) {
            return new ceres::DynamicAutoDiffCostFunction<So3TailFactor>(new So3TailFactor(weight));
        }

        static std::size_t TypeHashCode() {
            return typeid(So3TailFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<Sophus::SO3<T> const> const SO3_firToW(parBlocks[0]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_sedToW(parBlocks[1]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_thdToW(parBlocks[2]);

            Sophus::SO3<T> SO3_firToSed = SO3_sedToW.inverse() * SO3_firToW;
            Sophus::SO3<T> SO3_SedToThd = SO3_thdToW.inverse() * SO3_sedToW;

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = weight * (SO3_SedToThd.inverse() * SO3_firToSed).log();

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif 
