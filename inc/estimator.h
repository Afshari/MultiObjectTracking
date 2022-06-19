#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <QObject>
#include <Eigen/Dense>
#include <typeinfo>
#include <cmath>

#include "inc/stats/stats.hpp"
#include "inc/state.h"
#include "inc/measurement_model.h"
#include "inc/measurement_constant_velocity.h"
#include "inc/transition_model.h"
#include "inc/transition_constant_velocity.h"
#include "inc/utils.h"

using std::vector;
using std::tuple;
using std::shared_ptr;
using std::make_shared;
using Eigen::Vector2d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::ArrayXi;

class Estimator : public QObject {
    Q_OBJECT
public:
    explicit Estimator(QObject *parent = nullptr);
    explicit Estimator(shared_ptr<MeasurementModel> measurementModel, shared_ptr<TransitionModel> transitionModel,
                     QObject *parent = nullptr);
    virtual tuple<shared_ptr<ArrayXi>, shared_ptr<MatrixXd>> ellipsoidalGating(
                                const State &state, const MatrixXd &z, float gating_size);
    virtual shared_ptr<VectorXd> predictedLikelihood(const State &state, const MatrixXd &z);
    virtual shared_ptr<State> momentMatching(const VectorXd &w, const VecState &states);
    virtual State momentMatching(const VectorXd &w, const shared_ptr<vector<State>>& states);
    virtual shared_ptr<State> predict(const State &state);
    virtual shared_ptr<State> update(const State &state, const MatrixXd &z);

    virtual shared_ptr<VectorXd> h(const VectorXd &x);
    virtual shared_ptr<MatrixXd> H(const VectorXd &x);
    virtual MatrixXd getR();

protected:
    shared_ptr<MeasurementModel> measurementModel;
    shared_ptr<TransitionModel>  transitionModel;

};

#endif // ESTIMATOR_H






