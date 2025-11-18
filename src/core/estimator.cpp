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

#include <utility>
#include "core/estimator.h"
#include "factor/imu_gyro_factor.hpp"
#include "factor/gravity_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/radar_factor.hpp"
#include "factor/bias_factor.hpp"
#include "factor/tail_factor.hpp"
#include "factor/spline_factor.hpp"
#include "factor/gravity_update_factor.hpp"
#include "factor/gravity_spline_factor.hpp"
#include "factor/rot_refine_factor.hpp"
#include "factor/rot_prior_factor.hpp"
#include "factor/leg_vel_factor.hpp"
#include "factor/vel_bias_factor.hpp"
#include "factor/gravity_stationary_factor.hpp"

namespace garlileo {

    std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(new ceres::EigenQuaternionManifold());
    std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(new ceres::SphereManifold<3>());

    ceres::Problem::Options Estimator::DefaultProblemOptions() {
        return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA) {
        auto defaultSolverOptions = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
                threadNum, toStdout, useCUDA
        );
        if (!useCUDA) {
            defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
        }
        defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
        return defaultSolverOptions;
    }

    Estimator::Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                         const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                         const std::shared_ptr<Eigen::Vector3d> &bg,
                         const std::shared_ptr<Eigen::Vector2d> &bv)
            : ceres::Problem(DefaultProblemOptions()), configor(std::move(configor)),
              splines(std::move(splines)), gravity(gravity), ba(ba), bg(bg), bv(bv) {}

    Estimator::Ptr Estimator::Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                                     const std::shared_ptr<Eigen::Vector3d> &gravity,
                                     const std::shared_ptr<Eigen::Vector3d> &ba,
                                     const std::shared_ptr<Eigen::Vector3d> &bg,
                                     const std::shared_ptr<Eigen::Vector2d> &bv) {
        return std::make_shared<Estimator>(configor, splines, gravity, ba, bg, bv);
     }

    ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, this, &summary);
        return summary;
    }

    void Estimator::AddGravKnotsData(std::vector<double *> &paramBlockVec,
                                   const Estimator::SplineBundleType::RdSplineType &spline,
                                   const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;
            
            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
                this->AddParameterBlock(data, 3, GRAVITY_MANIFOLD.get());
                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }

    void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                                   const Estimator::SplineBundleType::RdSplineType &spline,
                                   const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;
            
            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

                this->AddParameterBlock(data, 3);
                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }

    void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                    const Estimator::SplineBundleType::So3SplineType &spline,
                                    const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
                // the local parameterization is very important!!!
                this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }


    void Estimator::AddVelPIMForGravityRecovery(const double dt, const Eigen::Vector3d &deltaVel,
                                                const Eigen::Vector3d &velPim, GaRLILEO_Opt option, double weight) {
        auto costFunc = GravityFactor::Create(dt, deltaVel, velPim, weight);

        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
    }

    Eigen::MatrixXd Estimator::CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix) {
        Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
        J.setZero();

        std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
        std::vector<double> jacobian_crs_matrix_values;
        jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
        jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
        jacobian_crs_matrix_values = jacobian_crs_matrix->values;

        int cur_index_in_cols_and_values = 0;
        // rows is a num_rows + 1 sized array
        int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
        // outer loop traverse rows, inner loop traverse cols and values
        for (int row_index = 0; row_index < row_size; ++row_index) {
            while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1]) {
                J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) =
                        jacobian_crs_matrix_values[cur_index_in_cols_and_values];
                cur_index_in_cols_and_values++;
            }
        }
        return J;
    }

    Eigen::MatrixXd Estimator::GetHessianMatrix() {
        ceres::Problem::EvaluateOptions EvalOpts;
        ceres::CRSMatrix jacobian_crs_matrix;
        this->Evaluate(EvalOpts, nullptr, nullptr, nullptr, &jacobian_crs_matrix);
        Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
        Eigen::MatrixXd H = J.transpose() * J;
        return H;
    }

    void Estimator::AddRadarMeasurement(const RadarTarget::Ptr &radarTar, GaRLILEO_Opt option, Sophus::SO3d& SO3_RefToW, double weight) {
        auto time = radarTar->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, velMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta); 
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta); 

        auto costFunc = RadarFactor<Configor::Prior::SplineOrder>::Create(
                configor->dataStream.CalibParam, SO3_RefToW, radarTar, so3Meta, velMeta, weight
        );

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(1);
        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_VEL, option)
        );

        this->AddResidualBlock(
                costFunc, new ceres::CauchyLoss(configor->prior.CauchyLossForRadarFactor * weight), paramBlockVec
        );
    }


    void Estimator::AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, GaRLILEO_Opt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta; 
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

        auto costFunc = IMUGyroFactorWithConstBias<Configor::Prior::SplineOrder>::Create(so3Meta, frame, weight);
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) { 
            costFunc->AddParameterBlock(4);
        }


        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_SO3, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    void Estimator::AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, GaRLILEO_Opt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }

        SplineMetaType so3Meta, velMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

        auto costFunc = IMUAcceFactorWithConstBias<Configor::Prior::SplineOrder>::Create(
                configor->dataStream.CalibParam, so3Meta, velMeta, frame, weight
        );
        

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // BIAS OF ACCE
        costFunc->AddParameterBlock(3);
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters() + 2);
        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_SO3, option)
        );
        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_VEL, option)
        );
        // BIAS OF ACCE
        paramBlockVec.push_back(ba->data());
        // gravity
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
        this->SetParameterBlockConstant(gravity->data());
    }

    void Estimator::ShowKnotStatus() const {
        for (const auto &[splineAddress, knotInfo]: knotRecoder) {
            std::stringstream stream;
            stream << "spline: " << splineAddress << ", ";
            for (const auto &[knotId, count]: knotInfo) {
                stream << '[' << knotId << ": " << count << "] ";
            }
            spdlog::info("{}", stream.str());
        }
    }

    ceres::ResidualBlockId Estimator::AddAcceBiasPriori(const BiasFilter::StatePack &priori, GaRLILEO_Opt option) {
        auto costFunc = BiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(3);
        costFunc->SetNumResiduals(3);
        auto id = this->AddResidualBlock(costFunc, nullptr, ba->data());
        
        if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        return id;
    }

    ceres::ResidualBlockId Estimator::AddVelBiasPriori(const VelBiasFilter::StatePack &priori, GaRLILEO_Opt option) {
        auto costFunc = VelBiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(2);
        costFunc->SetNumResiduals(2);
        auto id = this->AddResidualBlock(costFunc, nullptr, bv->data());
        
        if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_BA, option)) {
            this->SetParameterBlockConstant(bv->data());
        }
        return id;
    }

    std::vector<ceres::ResidualBlockId> Estimator::AddVelSplineTailConstraint(GaRLILEO_Opt option, double weight) {
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = RdTailFactor::Create(weight);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = velSpline.GetKnot(
                        j + i + static_cast<int>(velSpline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();

            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_VEL, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    std::vector<ceres::ResidualBlockId> Estimator::AddGravSplineTailConstraint(GaRLILEO_Opt option, int old_size, double weight){
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = RdTailFactor::Create(weight);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = gravSpline.GetKnot(
                        old_size + i
                ).data();

            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_VEL, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }

        }
        return idVec;
    }


    std::vector<ceres::ResidualBlockId> Estimator::AddSo3SplineTailConstraint(GaRLILEO_Opt option, double weight) {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = So3TailFactor::Create(weight);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = so3Spline.GetKnot(
                        j + i + static_cast<int>(so3Spline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();
            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            for (const auto &item: paramBlockVec) { this->SetManifold(item, QUATER_MANIFOLD.get()); }

            if (!GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_SO3, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    ceres::ResidualBlockId Estimator::AddGravityRotConstraint(double st, double et, const Eigen::Vector3d &alpha, 
                                            Sophus::SO3d& rot1, Sophus::SO3d& rot2, double weight) {
        if (!splines->TimeInRangeForRd(st, Configor::Preference::VelSpline) ||
            !splines->TimeInRangeForRd(et, Configor::Preference::VelSpline) ||
            !splines->TimeInRangeForRd(st, Configor::Preference::GravitySpline) ||
            !splines->TimeInRangeForRd(et, Configor::Preference::GravitySpline)) {
            return nullptr;
        }

        SplineMetaType gravMeta, velMeta;
        splines->CalculateRdSplineMeta(Configor::Preference::GravitySpline, {{st, st}}, gravMeta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{st, st},{et, et}}, velMeta);

        auto costFunc = GravityUpdateFactor<Configor::Prior::SplineOrder>::Create(
                gravMeta, velMeta, st, et, alpha, rot1, rot2, weight
        );

        // local grav
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(gravMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(gravMeta.NumParameters() + velMeta.NumParameters());

        AddGravKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::GravitySpline), gravMeta,
                false
        );

        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                false
        );
        // knots of vel spline
        

        auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        
        return id;
    }

    ceres::ResidualBlockId Estimator::AddGravitySplineConstraint(const IMUFrame::Ptr &frame, double weight){

        auto time = frame->GetTimestamp();
        if (!splines->TimeInRangeForRd(time, Configor::Preference::GravitySpline)) {
            return nullptr;
        }

        SplineMetaType so3Meta, gravMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::GravitySpline, {{time, time}}, gravMeta);

        auto costFunc = GravitySplineFactor<Configor::Prior::SplineOrder>::Create(gravMeta, so3Meta, frame, weight);
        

        for (int i = 0; i < static_cast<int>(gravMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }


        costFunc->SetNumResiduals(3);

        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(gravMeta.NumParameters() + so3Meta.NumParameters());

        AddGravKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::GravitySpline), gravMeta,
                false
        );

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                false
        );

        auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        return id;

    }

    void Estimator::AddRotRefine(long idx, double weight) {
        
        auto costFunc = RotRefineFactor<Configor::Prior::SplineOrder>::Create(weight);
        
        // Param for rotation   
        costFunc->AddParameterBlock(4);
        // knots of local gravity spline
        costFunc->AddParameterBlock(3);
        // Param for global gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(3);

        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        
        auto *data = const_cast<double *>(so3Spline.GetKnot(static_cast<int>(idx)).data());
        this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());
        paramBlockVec.push_back(data);
        // knot recoder
        knotRecoder[reinterpret_cast<long>(&so3Spline)][idx]++;

        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        auto *grav_data = const_cast<double *>(gravSpline.GetKnot(static_cast<int>(idx)).data());
        this->AddParameterBlock(grav_data, 3, GRAVITY_MANIFOLD.get());
        paramBlockVec.push_back(grav_data);
        this->SetParameterBlockConstant(grav_data);
        // knot recoder
        knotRecoder[reinterpret_cast<long>(&grav_data)][idx]++;
  
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());
        this->SetParameterBlockConstant(gravity->data());

        return;       
        
    }

    void Estimator::AddRotPrior(double st, double dt, double weight) {

        if (!splines->TimeInRangeForSo3(st, Configor::Preference::SO3Spline)) {
            return;
        }
        
        // prepare metas for splines
        SplineMetaType so3Meta, gravMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{st,st},
                                                                          {st+dt, st+dt}}, so3Meta);

        auto costFunc = RotPriorFactor<Configor::Prior::SplineOrder>::Create(st, dt, so3Meta, weight);

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                false
        );

        
        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        return;       
        
    }

    void Estimator::AddLegMeasurement(const LegFrame::Ptr &frame, GaRLILEO_Opt option, Sophus::SO3d& SO3_RefToW, double weight) {
        Eigen::Vector3d leg_vel = frame->GetVel();
        if(leg_vel(2)>0.75){
            return;
        }


        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            return;
        }

        SplineMetaType so3Meta, velMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

        auto costFunc = LegVelFactor<Configor::Prior::SplineOrder>::Create(configor->dataStream.CalibParam, so3Meta, velMeta, frame, SO3_RefToW ,weight);

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        //add velocity bias
        costFunc->AddParameterBlock(2);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters()+velMeta.NumParameters()+1);

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                false
        );

        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !GaRLILEO_OptOption::IsOptionWith(GaRLILEO_Opt::OPT_VEL, option)
        );

        paramBlockVec.push_back(bv->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        return;
    }

    void Estimator::AddStationaryGravity(Eigen::Vector3d &acceMean) {
        auto costFunc = GravityStationaryFactor::Create(acceMean, 10);
        // // organize the param block vector
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());
        return;
    }

    const std::map<long, std::map<std::size_t, int>>& Estimator::getKnotRecoder() const {
        return knotRecoder;
    }
}
