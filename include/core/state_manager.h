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

#ifndef GARLILEO_STATE_MANAGER_H
#define GARLILEO_STATE_MANAGER_H

#include <utility>
#include "core/data_manager.h"
#include "core/estimator.h"
#include "ctraj/factor/marginalization_factor.h"
#include "core/status.h"
#include "core/bias_filter.h"

namespace garlileo {
#define LOCK_STATES std::unique_lock<std::mutex> statesLock(StateManager::StatesMutex);

    class StateManager {
    public:
        using Ptr = std::shared_ptr<StateManager>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        struct StatePack {
        public:
            double timestamp{};
            Sophus::SO3d SO3_CurToRef;
            Eigen::Vector3d LIN_VEL_CurToRefInCur;
            Eigen::Vector3d gravity;
            Eigen::Vector3d ba;
            Eigen::Vector3d bg;
            Eigen::Vector2d bv;

            StatePack(double timestamp, const Sophus::SO3d &so3CurToRef, Eigen::Vector3d linVelCurToRefInCur,
                      Eigen::Vector3d gravity, Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector2d bv);

            StatePack();

            [[nodiscard]] Eigen::Vector3d LIN_VEL_CurToRefInRef() const;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(timestamp), CEREAL_NVP(SO3_CurToRef),
                        CEREAL_NVP(LIN_VEL_CurToRefInCur), CEREAL_NVP(gravity),
                        CEREAL_NVP(ba), CEREAL_NVP(bg), CEREAL_NVP(bv)
                );
            }
        };

    private:
        DataManager::Ptr dataMagr;
        Configor::Ptr configor;

        // spline bundles that keep four b-splines, i.e., the rotation spline, velocity spline,
        // and bias splines of accelerator and gyroscope
        SplineBundleType::Ptr splines;
        // the gravity represented in the world frame
        std::shared_ptr<Eigen::Vector3d> gravity;
        // the biases of acceleration and gyroscope
        std::shared_ptr<Eigen::Vector3d> ba, bg;
        std::shared_ptr<Eigen::Vector2d> bv;

        ns_ctraj::MarginalizationInfo::Ptr margInfo;

        BiasFilter::Ptr baFilter;
        VelBiasFilter::Ptr bvFilter;

        Sophus::SO3d SO3_RefToW;

        std::map<int, double *> lastKeepSo3KnotAdd;
        std::map<int, double *> lastKeepVelKnotAdd;
        std::map<int, double *> lastKeepGravKnotAdd;

    public:
        // mutexes employed in multi-thread framework
        static std::mutex StatesMutex;

    public:
        StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor);

        static Ptr Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor);

        void Run();

        const Sophus::SO3d& GetSO3_RefToW() const { return SO3_RefToW; }

        [[nodiscard]] std::optional<StatePack> GetStatePackSafely(double t) const;

        [[nodiscard]] std::vector<std::optional<StatePack>> GetStatePackSafely(const std::vector<double> &times) const;

        [[nodiscard]] const SplineBundleType::Ptr &GetSplines() const;

        [[nodiscard]] const BiasFilter::Ptr &GetBaFilter() const;

    protected:
        // --------------
        // initialization
        // --------------
        Sophus::SO3d ObtainAlignedWtoRef(const Sophus::SO3d &SO3_B0ToRef, const Eigen::Vector3d &gravityInRef);

        bool TryPerformInitialization();

        [[nodiscard]] inline SplineBundleType::Ptr CreateSplines(double sTime, double eTime) const;

        void InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData);

        void InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                               const std::list<IMUFrame::Ptr> &imuData);

        Estimator::Ptr InitializeVelSpline(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                           const std::list<IMUFrame::Ptr> &imuData,
                                           const std::list<LegFrame::Ptr> &legData);

        void AlignInitializedStates();

        void MarginalizationInInit(const Estimator::Ptr &estimator);

        void InitializeBiasFilters(double sTime);

        void UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime);

        bool IncrementalOptimization(const GaRLILEOStatus::StatusPack &status);


        void PreOptimization(const std::list<IMUFrame::Ptr> &imuData, double stime, double etime);

        void PostOptimization(long start_idx, long end_idx);

        void InitGravitySpline(const std::list<IMUFrame::Ptr> &imuData);

        static auto ExtractRange(const std::list<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        void ShowSplineStatus() const;

        template<class SplineType>
        void MarginalizeKnotsToSet(std::set<double *> &blocks, int oldSize, int newSize, SplineType &spline) {
            for (int i = std::max(0, oldSize - 2 * Configor::Prior::SplineOrder + 1);
                 i <= newSize - 2 * Configor::Prior::SplineOrder; ++i) {
                blocks.insert(spline.GetKnot(i).data());
            }
        }

        template<int Dime1, int Dime2>
        inline std::optional<Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor>>
        ObtainVarMatFromEstimator(const std::pair<const double *, const double *> &par, const Estimator::Ptr &est) {
            // compute the covariance of a parameter pair based on the estimator
            ceres::Covariance covariance({});
            auto res = covariance.Compute({par}, est.get());
            if (res) {
                Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor> cov;
                covariance.GetCovarianceBlock(par.first, par.second, cov.data());
                return cov;
            } else {
                return {};
            }
        }

        static void LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t);

        static void LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t);
    };
}


#endif
