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

#include "core/bias_filter.h"
#include <utility>

namespace garlileo {

    BiasFilter::StatePack::StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varVec)
            : time(time), state(std::move(state)) {
        this->var.setZero();
        this->var.diagonal() = varVec;
    }

    BiasFilter::StatePack::StatePack() = default;

    std::ostream &operator<<(std::ostream &os, const BiasFilter::StatePack &pack) {
        os << "time: " << pack.time
           << " state: " << pack.state.transpose()
           << " var: " << pack.var.diagonal().transpose();
        return os;
    }

    BiasFilter::BiasFilter(BiasFilter::StatePack init, double randomWalk)
            : curState(std::move(init)), sigma2(randomWalk * randomWalk) {
        stateRecords.push_back(curState);
    }

    BiasFilter::StatePack BiasFilter::Prediction(double t) const {
        StatePack predState;
        predState.time = t;

        // state propagation
        predState.state = curState.state;

        // covariance propagation
        predState.var = curState.var + (t - curState.time) * sigma2 * Eigen::Matrix3d::Identity();
        return predState;
    }

    const BiasFilter::StatePack &BiasFilter::GetCurState() const {
        return curState;
    }

    void BiasFilter::Update(const BiasFilter::StatePack &mes) {
        // prediction
        auto pred = Prediction(mes.time);

        // update
        Eigen::Matrix3d KMat = pred.var * (pred.var + mes.var).inverse();
        curState.time = mes.time;

        // state update
        curState.state = pred.state + KMat * (mes.state - pred.state);

        // covariance update
        Eigen::Matrix3d IKMat = (Eigen::Matrix3d::Identity() - KMat);
        curState.var = IKMat * pred.var * IKMat.transpose() + KMat * mes.var * KMat.transpose();

        stateRecords.push_back(curState);
    }

    BiasFilter::Ptr BiasFilter::Create(const BiasFilter::StatePack &init, double randomWalk) {
        return std::make_shared<BiasFilter>(init, randomWalk);
    }

    void BiasFilter::UpdateByEstimator(const BiasFilter::StatePack &est) {
        this->curState = est;
        stateRecords.push_back(curState);
    }

    const std::list<BiasFilter::StatePack> &BiasFilter::GetStateRecords() const {
        return stateRecords;
    }

    VelBiasFilter::StatePack::StatePack(double time, Eigen::Vector2d state, const Eigen::Vector2d &varVec)
            : time(time), state(std::move(state)) {
        this->var.setZero();
        this->var.diagonal() = varVec;
    }

    VelBiasFilter::StatePack::StatePack() = default;

    std::ostream &operator<<(std::ostream &os, const VelBiasFilter::StatePack &pack) {
        os << "time: " << pack.time
           << " state: " << pack.state.transpose()
           << " var: " << pack.var.diagonal().transpose();
        return os;
    }

    VelBiasFilter::VelBiasFilter(VelBiasFilter::StatePack init, double randomWalk)
            : curState(std::move(init)), sigma2(randomWalk * randomWalk) {
        stateRecords.push_back(curState);
    }

    VelBiasFilter::StatePack VelBiasFilter::Prediction(double t) const {
        StatePack predState;
        predState.time = t;

        // state propagation
        predState.state = curState.state;

        // covariance propagation
        predState.var = curState.var + (t - curState.time) * sigma2 * Eigen::Matrix2d::Identity();
        return predState;
    }

    const VelBiasFilter::StatePack &VelBiasFilter::GetCurState() const {
        return curState;
    }

    VelBiasFilter::Ptr VelBiasFilter::Create(const VelBiasFilter::StatePack &init, double randomWalk) {
        return std::make_shared<VelBiasFilter>(init, randomWalk);
    }

    void VelBiasFilter::UpdateByEstimator(const VelBiasFilter::StatePack &est) {
        this->curState = est;
        stateRecords.push_back(curState);
    }

    const std::list<VelBiasFilter::StatePack> &VelBiasFilter::GetStateRecords() const {
        return stateRecords;
    }
}