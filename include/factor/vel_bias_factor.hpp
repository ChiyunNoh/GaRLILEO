// GaRLILEO — Gravity-aligned Radar-Leg-Inertial Enhanced Odometry
// SPDX-License-Identifier: MIT
// © 2025 Chiyun Noh, Sangwoo Jung, Hanjun Kim, Yafei Hu, Laura Herlant, Ayoung Kim
// See LICENSE for the full MIT License text.

#ifndef VEL_BIAS_FACTOR_HPP
#define VEL_BIAS_FACTOR_HPP

namespace garlileo {

    struct VelBiasFactor {
    private:
        Eigen::Vector2d biasPriori;
        Eigen::Matrix2d weight;

    public:
        VelBiasFactor(Eigen::Vector2d biasPriori, const Eigen::Matrix2d &var) : biasPriori(std::move(biasPriori)) {
            weight = Eigen::LLT<Eigen::Matrix2d>(var.inverse()).matrixL().transpose();
        }

        static auto Create(const Eigen::Vector2d &biasPriori, const Eigen::Matrix2d &var) {
            return new ceres::DynamicAutoDiffCostFunction<VelBiasFactor>(new VelBiasFactor(biasPriori, var));
        }

        static std::size_t TypeHashCode() {
            return typeid(VelBiasFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector2<T>> bias(parBlocks[0]);

            Eigen::Map<Eigen::Vector2<T>> residuals(sResiduals);
            residuals = bias - biasPriori;
            residuals = weight * residuals;

            return true;
        }
    };
}

#endif
