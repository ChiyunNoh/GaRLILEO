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

#ifndef GARLILEO_ESTIMATOR_H
#define GARLILEO_ESTIMATOR_H

#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"
#include "ctraj/core/trajectory_estimator.h"
#include "sensor/imu.h"
#include "sensor/radar.h"
#include "core/bias_filter.h"
#include "sensor/leg_data_loader.h"

namespace garlileo {
    using namespace magic_enum::bitwise_operators;

    struct GaRLILEO_OptOption {
        enum class Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            OPT_SO3 = 1 << 1,
            OPT_VEL = 1 << 2,
            OPT_BA = 1 << 3,
            OPT_BG = 1 << 4,
            OPT_GRAVITY = 1 << 5,
            ALL = OPT_SO3 | OPT_VEL | OPT_BA | OPT_BG | OPT_GRAVITY
        };

        static bool IsOptionWith(Option desired, Option curOption) {
            return (desired == (desired & curOption));
        }
    };

    using GaRLILEO_Opt = GaRLILEO_OptOption::Option;

    class Estimator : public ceres::Problem {
    public:
        using Ptr = std::shared_ptr<Estimator>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        Configor::Ptr configor;
        SplineBundleType::Ptr splines;
        std::shared_ptr<Eigen::Vector3d> gravity;
        std::shared_ptr<Eigen::Vector3d> ba, bg;
        std::shared_ptr<Eigen::Vector2d> bv;

        // manifolds
        static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
        static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

        // involved knots recoder: [spline, knot, count]
        std::map<long, std::map<std::size_t, int>> knotRecoder;

    public:
        Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                  const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                  const std::shared_ptr<Eigen::Vector3d> &bg, const std::shared_ptr<Eigen::Vector2d> &bv);

        static Ptr Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                          const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                          const std::shared_ptr<Eigen::Vector3d> &bg, const std::shared_ptr<Eigen::Vector2d> &bv);

        static ceres::Problem::Options DefaultProblemOptions();

        static ceres::Solver::Options
        DefaultSolverOptions(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        ceres::Solver::Summary Solve(const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

        Eigen::MatrixXd GetHessianMatrix();

        void ShowKnotStatus() const;

    public:
        void AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, GaRLILEO_Opt option, double weight);

        void AddVelPIMForGravityRecovery(double dt, const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &velPim,
                                         GaRLILEO_Opt option, double weight);

        void AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, GaRLILEO_Opt option, double weight);

        void AddRadarMeasurement(const RadarTarget::Ptr &radarTar, GaRLILEO_Opt option, Sophus::SO3d& SO3_RefToW, double weight);

        ceres::ResidualBlockId AddAcceBiasPriori(const BiasFilter::StatePack &priori, GaRLILEO_Opt option);

        ceres::ResidualBlockId AddGyroBiasPriori(const BiasFilter::StatePack &priori, GaRLILEO_Opt option);

        ceres::ResidualBlockId AddVelBiasPriori(const VelBiasFilter::StatePack &priori, GaRLILEO_Opt option);

        std::vector<ceres::ResidualBlockId> AddVelSplineTailConstraint(GaRLILEO_Opt option, double weight);

        std::vector<ceres::ResidualBlockId> AddSo3SplineTailConstraint(GaRLILEO_Opt option, double weight);

        std::vector<ceres::ResidualBlockId> AddGravSplineTailConstraint(GaRLILEO_Opt option, int old_size, double weight);

        void AddStationaryGravity(Eigen::Vector3d &acceMean);


        ceres::ResidualBlockId AddGravityRotConstraint(double t1, double t2, const Eigen::Vector3d &alpha, Sophus::SO3d& rot1, Sophus::SO3d& rot2, double weight);

        ceres::ResidualBlockId AddGravitySplineConstraint(const IMUFrame::Ptr &frame, double weight);

        void AddRotRefine(long idx, double weight);

        void AddRotPrior(double st, double dt, double weight);
        
        void AddLegMeasurement(const LegFrame::Ptr &frame, GaRLILEO_Opt option, Sophus::SO3d& SO3_RefToW, double weight);

        const std::map<long, std::map<std::size_t, int>>& getKnotRecoder() const ;
    protected:
        void AddSo3KnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::So3SplineType &spline,
                             const SplineMetaType &splineMeta, bool setToConst);

        void AddRdKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                            const SplineMetaType &splineMeta, bool setToConst);

        void AddGravKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                            const SplineMetaType &splineMeta, bool setToConst);

        static Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix);
    };
}

#endif //GARLILEO_ESTIMATOR_H
