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

#include "core/state_manager.h"

#include <utility>
#include "core/estimator.h"
#include "spdlog/stopwatch.h"
 

namespace garlileo {

    // -------------------
    // static member field
    // -------------------
    std::mutex StateManager::StatesMutex = {};
    double st = 0;
    long update_start_index = -1; 
    long update_end_index = -1; 

    StateManager::StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor)
            : dataMagr(std::move(dataMagr)), configor(std::move(configor)), splines(nullptr),
              gravity(std::make_shared<Eigen::Vector3d>(0.0, 0.0, -this->configor->prior.GravityNorm)),
              ba(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)),
              bg(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)), 
              bv(std::make_shared<Eigen::Vector2d>(0.0, 0.0)), margInfo(nullptr) {}

    StateManager::Ptr StateManager::Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor) {
        return std::make_shared<StateManager>(dataMagr, configor);
    }

    void StateManager::Run() {
        spdlog::info("'StateManager::Run' has been booted, thread id: {}.", GARLILEO_TO_STR(std::this_thread::get_id()));

        rclcpp::Rate rate(configor->preference.IncrementalOptRate);

        while (rclcpp::ok()) {
            auto status = GaRLILEOStatus::GetStatusPackSafely();
            if (GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("'StateManager::Run' quits normally.");
                break;
            }

            if (!GaRLILEOStatus::IsWith(GaRLILEOStatus::StateManager::Status::HasInitialized, status.StateMagr)) { 
                // if this system has not been initialized
                if (GaRLILEOStatus::IsWith(GaRLILEOStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    // if this system is ready for initialization
                    {
                        LOCK_GARLILEO_STATUS
                        GaRLILEOStatus::DataManager::CurStatus ^= GaRLILEOStatus::DataManager::Status::RadarTarAryForInitIsReady;
                     }
                    spdlog::stopwatch sw;
                    auto valid = TryPerformInitialization();
                    if (valid) { spdlog::info("total time elapsed in initialization: {} (s).", sw); }
                } else {
                    spdlog:: info("preparing initialization...");
                }
            } else {
                spdlog::stopwatch sw;
                auto valid = IncrementalOptimization(status);
                if (valid) {
                    // spdlog::info("total time elapsed in current incremental optimization: {} (s). \n", sw);
                }
            }

            rate.sleep();
        }
    }

    // obtain the rotation from aligned {w} to {ref} based on 'SO3_B0ToRef' and 'gravityInRef'
    Sophus::SO3d StateManager::ObtainAlignedWtoRef(const Sophus::SO3d &SO3_B0ToRef, const Eigen::Vector3d &gravityInRef) {
        Eigen::Vector3d zNegAxis = configor->prior.GravityDirection * SO3_B0ToRef.matrix().col(2); 
        Eigen::Vector3d rotDir = zNegAxis.cross(gravityInRef).normalized();
        double angRad = std::acos(zNegAxis.dot(gravityInRef) / gravityInRef.norm());
        Sophus::SO3d SO3_WtoRef = Sophus::SO3d(Eigen::AngleAxisd(angRad, rotDir).toRotationMatrix()) * SO3_B0ToRef;
        return SO3_WtoRef;
    }

    bool StateManager::TryPerformInitialization() {
        auto radarTarAryForInit = dataMagr->GetRadarTarAryForInitSafely();

        // get start and end time of radar target arrays clocked by the IMU
        double sTimeRadar = radarTarAryForInit.front()->GetTargets().front()->GetTimestamp();
        double eTimeRadar = radarTarAryForInit.back()->GetTargets().back()->GetTimestamp();

        auto imuDataSeq = dataMagr->ExtractIMUDataPieceSafely(sTimeRadar, eTimeRadar);

        auto legDataSeq = dataMagr->ExtractLegDataPieceSafely(sTimeRadar, eTimeRadar);

        // ----------------
        // static condition
        // ----------------
        double sTime = std::max(sTimeRadar, imuDataSeq.front()->GetTimestamp());
        double eTime = std::min(eTimeRadar, imuDataSeq.back()->GetTimestamp());

        spdlog::info(
                "'initialization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                sTime, eTime, eTime - sTime
        );

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info(
                    "'radarTarAryForInit' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    radarTarAryForInit.size(), sTimeRadar, eTimeRadar, sTimeRadar - eTimeRadar
            );
            spdlog::info(
                    "involved IMU frame size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    imuDataSeq.size(), imuDataSeq.front()->GetTimestamp(), imuDataSeq.back()->GetTimestamp(),
                    imuDataSeq.back()->GetTimestamp() - imuDataSeq.front()->GetTimestamp()
            );
            spdlog::info(
                    "involved Leg frame size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    legDataSeq.size(), legDataSeq.front()->GetTimestamp(), legDataSeq.back()->GetTimestamp(),
                    legDataSeq.back()->GetTimestamp() - legDataSeq.front()->GetTimestamp()
            );
        }

        // create splines
        splines = CreateSplines(sTime, eTime);
        if (Configor::Preference::DEBUG_MODE) { ShowSplineStatus(); }

        // init filters
        InitializeBiasFilters(sTime);

        // ---------------------
        // initialize so3 spline
        // ---------------------
        spdlog::stopwatch sw;
        InitializeSO3Spline(imuDataSeq);
        spdlog::info("initialized rotation spline time elapsed: {} (s).", sw);

        // -------------------------
        // initialize global gravity vector
        // -------------------------
        spdlog::stopwatch sw2;
        InitializeGravity(radarTarAryForInit, imuDataSeq);

        // observability condition is good, the gravity has been initialized
        spdlog::info(
                "initialized gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // -----------------------
        // recover velocity spline
        // -----------------------
        spdlog::stopwatch sw3;
        
        auto estimator = InitializeVelSpline(radarTarAryForInit, imuDataSeq, legDataSeq);

        spdlog::info("initialize velocity spline time elapsed: {} (s)", sw3);


        spdlog::info(
                "refined gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ------------------
        // update bias filter
        // ------------------
        UpdateBiasFilters(estimator, eTime);

        // -----------------------
        // align states to gravity
        // -----------------------
        AlignInitializedStates();

        spdlog::info(
                "aligned gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ---------------------------------
        // marginalization in initialization
        // ---------------------------------
        spdlog::stopwatch sw4;
        
        MarginalizationInInit(estimator);
        
        spdlog::info("marginalization in initialization time elapsed: {} (s)", sw4);
        
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        std::size_t num_knots = 0;
        while (num_knots<so3Spline.GetKnots().size()) {
            gravSpline.GetKnot(num_knots) = (so3Spline.GetKnot(num_knots).matrix().transpose()) * (*gravity);
            num_knots++;
        }

        LOCK_GARLILEO_STATUS
        GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::HasInitialized;
        GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::NewStateNeedToDraw;
        GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::NewStateNeedToPublish;
        GaRLILEOStatus::StateManager::ValidStateEndTime = eTime;

        return true;
    }

    StateManager::SplineBundleType::Ptr StateManager::CreateSplines(double sTime, double eTime) const {
        // create four splines
        auto so3SplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::SO3Spline, ns_ctraj::SplineType::So3Spline,
                sTime, eTime, configor->prior.SO3SplineKnotDist
        );
        auto velSplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::VelSpline, ns_ctraj::SplineType::RdSpline,
                sTime, eTime, configor->prior.VelSplineKnotDist
        );

        auto gravitySplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::GravitySpline, ns_ctraj::SplineType::RdSpline,
                sTime, eTime, configor->prior.GravSplineKnotDist
        );
        return SplineBundleType::Create({so3SplineInfo, velSplineInfo, gravitySplineInfo});
    }

    void StateManager::InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData) {
        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, GaRLILEO_Opt::OPT_SO3, configor->prior.GyroWeight);
        } 

        // make this problem fun rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeSO3Spline':\n{}\n", sum.BriefReport());
        }
    }

    void StateManager::InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                         const std::list<IMUFrame::Ptr> &imuData) {
        // obtain extrinsics
        const auto &SO3_RtoB = configor->dataStream.CalibParam.SO3_RtoB;
        const Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

        // obtain the fitted so3 spline
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);

        // timestamp, imu velocity computed from radar measurements
        std::vector<std::pair<double, Eigen::Vector3d>> LIN_VEL_BtoB0inB0_VEC;

        // compute the velocity of imu (with respect to the reference frame expressed in the reference frame)
        for (const auto &tarAry: radarTarAryVec) {
            double t = tarAry->GetTimestamp();

            if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }

            auto SO3_BtoB0 = so3Spline.Evaluate(t);
            Eigen::Vector3d ANG_VEL_BtoB0inB0 = SO3_BtoB0 * so3Spline.VelocityBody(t);

            Sophus::SO3d SO3_RtoB0 = SO3_BtoB0 * SO3_RtoB;
            Eigen::Vector3d LIN_VEL_RtoB0InB0 = tarAry->RadarVelocityFromStaticTargetArray(SO3_RtoB0);

            Eigen::Vector3d LIN_VEL_BtoB0inB0 =
                    LIN_VEL_RtoB0InB0 + Sophus::SO3d::hat(SO3_BtoB0 * POS_RinB) * ANG_VEL_BtoB0inB0;

            LIN_VEL_BtoB0inB0_VEC.emplace_back(t, LIN_VEL_BtoB0inB0);
        }
        Eigen::Vector3d mean;
        Eigen::MatrixXd var;
        Eigen::MatrixXd matrix(LIN_VEL_BtoB0inB0_VEC.size(), 3);
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);
        for (int i = 0; i < static_cast<int>(LIN_VEL_BtoB0inB0_VEC.size()) - 1; ++i) {
            int j = i + 1;
            const auto &[ti, vi] = LIN_VEL_BtoB0inB0_VEC.at(i);
            const auto &[tj, vj] = LIN_VEL_BtoB0inB0_VEC.at(j);
            matrix.row(i++) = vi;
            auto [sIter, eIter] = ExtractRange(imuData, ti, tj);
            std::vector<std::pair<double, Eigen::Vector3d>> velData;
            for (auto iter = sIter; iter != eIter; ++iter) {
                const auto &frame = *iter;
                double t = frame->GetTimestamp();
                velData.emplace_back(t, so3Spline.Evaluate(t) * frame->GetAcce());
            }
            Eigen::Vector3d velPIM = TrapIntegrationOnce(velData);
            estimator->AddVelPIMForGravityRecovery(tj - ti, vj - vi, velPIM, GaRLILEO_Opt::OPT_GRAVITY, 1.0);
        }

        if (LIN_VEL_BtoB0inB0_VEC.size() != 0) {
            mean = matrix.colwise().mean();
            var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                                (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(LIN_VEL_BtoB0inB0_VEC.size() - 1);
        }
        
        if (mean.norm()<0.01 && var.diagonal().norm() < 0.01){
            auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuData);
            estimator->AddStationaryGravity(acceMean);
        }
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeGravity':\n{}\n", sum.BriefReport());
        }

    }

    Estimator::Ptr StateManager::InitializeVelSpline(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                                     const std::list<IMUFrame::Ptr> &imuData,
                                                     const std::list<LegFrame::Ptr> &legData) {

        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);

        // optimization option in initialization
        GaRLILEO_Opt option = GaRLILEO_Opt::OPT_SO3 | GaRLILEO_Opt::OPT_VEL | GaRLILEO_Opt::OPT_GRAVITY;

        // acce and gyro factors
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeight);
            estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeight);
        }

        // radar factors
        for (const auto &ary: radarTarAryVec) {
            for (const auto &tar: ary->GetTargets()) {
                estimator->AddRadarMeasurement(tar, option, SO3_RefToW, configor->prior.RadarWeight);
            }
        }

        for (const auto &frame: legData) {
            estimator->AddLegMeasurement(frame, option, SO3_RefToW, configor->prior.LegWeight);
        }

        // add a tail constraint to handle the poor observability of the last knot
        auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

        // make this problem fun rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

        option |= GaRLILEO_Opt::OPT_BA;
        sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeVelSpline':\n{}\n", sum.BriefReport());
        }

        for (const auto &id: velTailIdVec) { estimator->RemoveResidualBlock(id); }
        for (const auto &id: so3TailIdVec) { estimator->RemoveResidualBlock(id); }

        return estimator;
    }

    void StateManager::InitializeBiasFilters(double sTime) {
        // ShowSplineStatus();
        {
            // -----------------
            // acceleration bias
            // -----------------
            // the covariance of the initial state is set to 0.01 m/s^2
            BiasFilter::StatePack initState(
                    sTime, *ba, Eigen::Vector3d::Ones() * 0.0001 * 0.0001
            );
            baFilter = BiasFilter::Create(initState, configor->prior.AcceBiasRandomWalk);
            spdlog::info("initial state of ba filter: {}", GARLILEO_TO_STR(baFilter->GetCurState()));

        }

        {

            VelBiasFilter::StatePack initState(
                    sTime, *bv, Eigen::Vector2d::Ones() * 0.0001 * 0.0001
            );
            bvFilter = VelBiasFilter::Create(initState, configor->prior.AcceBiasRandomWalk);
            spdlog::info("initial state of bv filter: {}", GARLILEO_TO_STR(bvFilter->GetCurState()));
        }
    }

    void StateManager::AlignInitializedStates() {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        // current gravity, velocities, and rotations are expressed in the reference frame
        // align them to the world frame whose negative z axis is aligned with the gravity vector
        spdlog::info(
                "aligned gravity vector1: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );
        SO3_RefToW = ObtainAlignedWtoRef(so3Spline.Evaluate(so3Spline.MinTime()), *gravity).inverse();
        *gravity = SO3_RefToW * *gravity;
        spdlog::info(
                "aligned gravity vector2: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );
        for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i) {
            so3Spline.GetKnot(i) = SO3_RefToW * so3Spline.GetKnot(i);
        }
        // for (int i = 0; i < static_cast<int>(velSpline.GetKnots().size()); ++i) {
        //     velSpline.GetKnot(i) = SO3_RefToW * velSpline.GetKnot(i);
        // }
        // const auto &SO3_RtoB = configor->dataStream.CalibParam.SO3_RtoB;
        // const Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

        // const auto &SO3_BtoL = configor->dataStream.CalibParam.SO3_BtoL;

        // configor->dataStream.CalibParam.SO3_RtoB = SO3_RefToW * SO3_RtoB;
        // configor->dataStream.CalibParam.POS_RinB = SO3_RefToW * POS_RinB;

        // configor->dataStream.CalibParam.SO3_BtoL = SO3_BtoL * SO3_RefToW.inverse();
    }

    void StateManager::MarginalizationInInit(const Estimator::Ptr &estimator) {
        // perform marginalization
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotSize = static_cast<int>(velSpline.GetKnots().size());

        std::set<double *> margParBlocks{gravity->data()};       
        for (int i = 0; i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = 0; i < velKnotSize; ++i) {
            if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(velSpline.GetKnot(i).data());
            } else {
                lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
            }
        }
        
        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("rotation knots: {}, velocity knots: {}", so3KnotSize, velKnotSize);
            spdlog::info(
                    "marginalize rotation knots from 0-th to {}-th", so3KnotSize - Configor::Prior::SplineOrder - 1
            );
            spdlog::info(
                    "marginalize velocity knots from 0-th to {}-th", velKnotSize - Configor::Prior::SplineOrder - 1
            );
        }
        spdlog::info("rotation knots12");
        margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 2);
    }

    void StateManager::PreOptimization(const std::list<IMUFrame::Ptr> &imuData, double stime, double etime) {
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);

        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, GaRLILEO_Opt::OPT_SO3, configor->prior.GyroWeight);
        }
        // maintain observability of the last few control points
        estimator->AddSo3SplineTailConstraint(GaRLILEO_Opt::OPT_SO3, 1000.0);

        double dt = 0.05;
        for (double t=stime; t<etime-0.05; t+=0.01){
            estimator->AddRotPrior(t, dt, 1000);
        }
        
        // solving
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));
    }

    void StateManager::InitGravitySpline(const std::list<IMUFrame::Ptr> &imuData){
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);
        GaRLILEO_Opt option = GaRLILEO_Opt::OPT_SO3 | GaRLILEO_Opt::OPT_VEL | GaRLILEO_Opt::OPT_BA;
        for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
            estimator->AddGravitySplineConstraint(*iter1, 100);
        }
        auto gravTailIdVec = estimator->AddGravSplineTailConstraint(option, gravSpline.GetKnots().size()-3, 1000.0);

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        for (const auto &id: gravTailIdVec) { estimator->RemoveResidualBlock(id); }

    }

    void StateManager::PostOptimization(long start_idx, long end_idx) {
        
        if(start_idx < 5){
            spdlog::info("not enough knots for update Rotation using Gravity");
            return;
        }

        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);

        for (long idx = start_idx-2; idx < end_idx-2; idx++){
            estimator->AddRotRefine(idx, 100);
        }

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, false, false)); 

        ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

    }

    bool StateManager::IncrementalOptimization(const GaRLILEOStatus::StatusPack &status) { 
        const double sTime = status.ValidStateEndTime, eTime = dataMagr->GetEndTimeSafely();

        if (eTime - sTime < 1.0 / configor->preference.IncrementalOptRate) { return false; }

        // spdlog::info(
        //         "'incremental optimization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
        //         sTime, eTime, eTime - sTime
        // );
        
        auto imuData = dataMagr->ExtractIMUDataPieceSafely(sTime, eTime);
        auto radarData = dataMagr->ExtractRadarDataPieceSafely(sTime, eTime);
        auto legDataSeq = dataMagr->ExtractLegDataPieceSafely(sTime, eTime);

        const double dt = eTime - sTime;
        const double imuFreq = static_cast<double>(imuData.size()) / dt;
        const double radarFreq = static_cast<double>(radarData.size()) / dt;

        if (imuFreq < 50 || radarFreq < 10) {
            LOCK_GARLILEO_STATUS
            GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::ShouldQuit;
            return false;
        }

        // ----------------
        // static condition
        // ----------------
        LOCK_STATES
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuData);
        double velocity = velSpline.Evaluate(status.ValidStateEndTime - 1E-3).norm();

        for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
            Eigen::Quaternion<double> ori = (*iter1)->GetOrientation();
            Sophus::SO3d so3(ori);
            Sophus::SO3d gravity_align = SO3_RefToW * so3;
            Eigen::Quaternion<double> quat_g_align = gravity_align.unit_quaternion();
            (*iter1)->SetOrientation(quat_g_align);

            // Eigen::Vector3d gyro((*iter1)->GetGyro());
            // Eigen::Vector3d gravity_align_gyro = SO3_RefToW * gyro;

            // (*iter1)->SetGyro(gravity_align_gyro);
        }
        

        // --------------
        // extend splines
        // --------------
        int so3KnotOldSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotOldSize = static_cast<int>(velSpline.GetKnots().size());
        int gravKnotOldSize = static_cast<int>(gravSpline.GetKnots().size());

        if (Configor::Preference::DEBUG_MODE) { ShowSplineStatus(); }

        // --------------------
        // linear extrapolation
        // --------------------
        LinearExtendKnotTo(so3Spline, eTime + 1E-6);
        LinearExtendKnotTo(velSpline, eTime + 1E-6);
        

        int old_grav_idx  = gravSpline.GetKnots().size();
        while (gravSpline.GetKnots().size()<so3Spline.GetKnots().size()) {
            gravSpline.KnotsPushBack((so3Spline.GetKnot(gravSpline.GetKnots().size()).matrix().transpose()) * (*gravity));
        }

        // obtain the updated address
        std::map<double *, double *> updatedLastKeepKnotAdd;
        for (const auto &[idx, add]: lastKeepSo3KnotAdd) {
            updatedLastKeepKnotAdd.insert({add, so3Spline.GetKnot(idx).data()});
        }
        for (const auto &[idx, add]: lastKeepVelKnotAdd) {
            updatedLastKeepKnotAdd.insert({add, velSpline.GetKnot(idx).data()});
        }
        for (const auto &[idx, add]: lastKeepGravKnotAdd) {
            updatedLastKeepKnotAdd.insert({add, gravSpline.GetKnot(idx).data()});
        }
        margInfo->ShiftKeepParBlockAddress(updatedLastKeepKnotAdd);

        // ------------------------
        // incremental optimization
        // ------------------------

        PreOptimization(imuData, sTime, eTime);

        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg, bv);
        // // marginalization factor
        ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

        GaRLILEO_Opt option = GaRLILEO_Opt::OPT_SO3 | GaRLILEO_Opt::OPT_VEL | GaRLILEO_Opt::OPT_BA;
        std::set<ceres::ResidualBlockId> gravIdSet;
        std::set<double*> gravParamSet;
        std::set<int> gravIDX;
        for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
            double t1 = (*iter1)->GetTimestamp();
            auto id1 = estimator->AddGravitySplineConstraint(*iter1, 1000);
            Eigen::Quaternion<double> ori1 = (*iter1)->GetOrientation();
            Sophus::SO3d rot1(ori1);
            estimator->AddGyroMeasurementWithConstBias(*iter1, option, configor->prior.GyroWeight);
            for (auto iter2 = std::next(iter1); iter2 != imuData.cend(); ++iter2) {
                Eigen::Quaternion<double> ori2 = (*iter2)->GetOrientation();
                Sophus::SO3d rot2(ori2);
                double t2 = (*iter2)->GetTimestamp();
                std::vector<std::pair<double, Eigen::Vector3d>> velData;                
                double before_t = t1;
                for (auto iter = iter1; iter != std::next(iter2); ++iter) { 
                    const auto &frame = *iter;
                    double t = frame->GetTimestamp();
                    velData.emplace_back(t, so3Spline.Evaluate(t1).matrix().transpose() * so3Spline.Evaluate(t).matrix() * (frame->GetAcce()-*ba));
                    before_t = t;
                }
                Eigen::Vector3d velPIM = TrapIntegrationOnce(velData);


                auto id1 = estimator->AddGravityRotConstraint(t1, t2 , velPIM, rot1, rot2, configor->prior.GravityWeight);
                gravIdSet.insert(id1);
            }
        }
        
        auto baPrioriId = estimator->AddAcceBiasPriori(baFilter->Prediction(eTime), option);

        auto bvPrioriId = estimator->AddVelBiasPriori(bvFilter->Prediction(eTime), option);

        // // radar factors
        for (const auto &tar: radarData) {
            estimator->AddRadarMeasurement(tar, option, SO3_RefToW, configor->prior.RadarWeight);
        }

        for (const auto &frame: legDataSeq) {
            estimator->AddLegMeasurement(frame, option, SO3_RefToW, configor->prior.LegWeight);
        }

        // // ---------------------------------------------------
        // // handle the poor observability of the last few knots
        // // ---------------------------------------------------
        std::size_t old_grav = 0;
        std::map<long, std::map<std::size_t, int>> knotRecoder = estimator->getKnotRecoder();
        for (const auto &[knotId, count]: knotRecoder[reinterpret_cast<long>(&gravSpline)]){
            if(old_grav == 0) old_grav = knotId;
            auto *data = const_cast<double *>(gravSpline.GetKnot(knotId).data());
            gravParamSet.insert(data);
            gravIDX.insert(knotId);
        }

        old_grav_idx = static_cast<int>(old_grav);
        auto gravTailIdVec = estimator->AddGravSplineTailConstraint(option, old_grav_idx, 1000.0);

        auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        // // ------------------
        // // update bias filter
        // // ------------------

        update_start_index = update_end_index;
        for (const auto &[knotId, count]: knotRecoder[reinterpret_cast<long>(&so3Spline)]){
            update_end_index = knotId;
            break;
        }
        UpdateBiasFilters(estimator, eTime);

        PostOptimization(update_start_index, update_end_index);

        // // -----------------------
        // // perform marginalization
        // // -----------------------

        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotSize = static_cast<int>(velSpline.GetKnots().size());
        int gravKnotSize = static_cast<int>(gravSpline.GetKnots().size());
        
        std::set<double *> margParBlocks{gravity->data()};
        lastKeepSo3KnotAdd.clear(), lastKeepVelKnotAdd.clear(), lastKeepGravKnotAdd.clear();
        for (int i = std::max(0, so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = std::max(0, velKnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < velKnotSize; ++i) {
            if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(velSpline.GetKnot(i).data());
            } else {
                lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
            }
        }

        for (int i = std::max(0, gravKnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < gravKnotSize; ++i) {
            if (i <= gravKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(gravSpline.GetKnot(i).data());
            } else {
                lastKeepGravKnotAdd.insert({i, gravSpline.GetKnot(i).data()});
            }
        }


        estimator->RemoveResidualBlock(baPrioriId);
        estimator->RemoveResidualBlock(bvPrioriId);
        // remove linear extrapolation factor as they are not come from real-sensor measurements
        for (const auto &id: velTailIdVec) { estimator->RemoveResidualBlock(id); }
        for (const auto &id: so3TailIdVec) { estimator->RemoveResidualBlock(id); }

        st = sTime;

        margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 1);

        LOCK_GARLILEO_STATUS
        GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::NewStateNeedToDraw;
        GaRLILEOStatus::StateManager::CurStatus |= GaRLILEOStatus::StateManager::Status::NewStateNeedToPublish;
        GaRLILEOStatus::StateManager::ValidStateEndTime = eTime;

        return true;
    }

    std::optional<StateManager::StatePack> StateManager::GetStatePackSafely(double t) const {
        LOCK_STATES
        if (splines == nullptr) { return {}; }

        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        if (!splines->TimeInRange(t, so3Spline)) { return {}; }

        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        if (!splines->TimeInRange(t, velSpline)) { return {}; }

        StatePack pack;
        pack.timestamp = t;
        pack.ba = *ba;
        pack.bg = *bg;
        pack.bv = *bv;
        pack.gravity = *gravity;
        pack.SO3_CurToRef = so3Spline.Evaluate(t);
        pack.LIN_VEL_CurToRefInCur = velSpline.Evaluate(t);

        return pack;
    }

    std::vector<std::optional<StateManager::StatePack>>
    StateManager::GetStatePackSafely(const std::vector<double> &times) const {
        std::vector<std::optional<StateManager::StatePack>> packs(times.size());
        LOCK_STATES
        if (splines == nullptr) { return packs; }
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);

        for (int i = 0; i < static_cast<int>(times.size()); ++i) {
            double t = times.at(i);
            if (!splines->TimeInRange(t, so3Spline) || !splines->TimeInRange(t, velSpline)) {
                packs.at(i) = {};
            } else {
                StatePack pack;
                pack.timestamp = t;
                pack.ba = *ba;
                pack.bg = *bg;
                pack.bv = *bv;
                pack.gravity = *gravity;
                pack.SO3_CurToRef = so3Spline.Evaluate(t);
                pack.LIN_VEL_CurToRefInCur = velSpline.Evaluate(t);

                packs.at(i) = pack;
            }
        }
        return packs;
    }

    const StateManager::SplineBundleType::Ptr &StateManager::GetSplines() const {
        return splines;
    }

    void StateManager::ShowSplineStatus() const {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        auto &gravSpline = splines->GetRdSpline(Configor::Preference::GravitySpline);
        spdlog::info(
                "[Rotation Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
                so3Spline.MinTime(), so3Spline.MaxTime(), so3Spline.GetTimeInterval(), so3Spline.GetKnots().size()
        );
        spdlog::info(
                "[Velocity Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
                velSpline.MinTime(), velSpline.MaxTime(), velSpline.GetTimeInterval(), velSpline.GetKnots().size()
        );
        spdlog::info(
                "[Gravity Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
                gravSpline.MinTime(), gravSpline.MaxTime(), gravSpline.GetTimeInterval(), gravSpline.GetKnots().size()
        );
    }

    void StateManager::UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime) {
        auto baCovOpt = ObtainVarMatFromEstimator<3, 3>({ba->data(), ba->data()}, estimator);
        if (baCovOpt != std::nullopt) {
            Eigen::Vector3d baCov = baCovOpt->diagonal();
            baFilter->UpdateByEstimator(BiasFilter::StatePack(eTime, *ba, baCov));
            // spdlog::info("current state of ba filter: {}", GARLILEO_TO_STR(baFilter->GetCurState()));
        }
        
        auto bvCovOpt = ObtainVarMatFromEstimator<2,2>({bv->data(), bv->data()}, estimator);
        
        if (bvCovOpt != std::nullopt) {
            Eigen::Vector2d bvCov = bvCovOpt->diagonal();
            bvFilter->UpdateByEstimator(VelBiasFilter::StatePack(eTime, *bv, bvCov));
            // spdlog::info("current state of bv filter: {}", GARLILEO_TO_STR(bvFilter->GetCurState()));
        }
    }

    const BiasFilter::Ptr &StateManager::GetBaFilter() const {
        return baFilter;
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t) {
        Eigen::Vector3d delta = spline.GetKnots().back() - spline.GetKnots().at(spline.GetKnots().size() - 2);
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() + delta);
        }
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t) {
        Sophus::SO3d delta =
                spline.GetKnots().at(spline.GetKnots().size() - 2).inverse() * spline.GetKnots().back();
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() * delta);
        }
    }

    StateManager::StatePack::StatePack(double timestamp, const Sophus::SO3d &so3CurToRef,
                                       Eigen::Vector3d linVelCurToRefInCur, Eigen::Vector3d gravity,
                                       Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector2d bv)
            : timestamp(timestamp), SO3_CurToRef(so3CurToRef), LIN_VEL_CurToRefInCur(std::move(linVelCurToRefInCur)),
              gravity(std::move(gravity)), ba(std::move(ba)), bg(std::move(bg)), bv(std::move(bv)) {}

    Eigen::Vector3d StateManager::StatePack::LIN_VEL_CurToRefInRef() const {
        return SO3_CurToRef * LIN_VEL_CurToRefInCur;
    }

    StateManager::StatePack::StatePack() = default;
}