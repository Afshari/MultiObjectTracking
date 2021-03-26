#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QObject>
#include <QTest>
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "measurementmodel.h"
#include "measurementprediction.h"
#include "transitionmodel.h"
#include "transitionlineargaussian.h"
#include "state.h"
#include "stategaussian.h"
#include "estimator.h"


class KalmanFilter : public Estimator
{
    Q_OBJECT
public:
    explicit KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel);

    virtual State* predict(const State &prior, int dt) override;
    virtual State* update(const State &state, const MeasurementModel &measurementModel,
                          const VectorXd &measurement, const MeasurementPrediction &measurementPrediction) override;
    virtual MeasurementPrediction *predictMeasurement(State *predState) override;

    friend class TestKalmanFilter;
    friend class DebugServer;

private:
//    VectorXd prior;
//    VectorXd xPred;
//    VectorXd pPred;
//    VectorXd xPosterior;
//    VectorXd pPosterior;

//    MeasurementModel *measurementModel;
//    TransitionModel  *transitionModel;

    VectorXd xPredict(const State &prior, int dt);
    MatrixXd PPredict(const State &prior, int dt);

    MatrixXd K(const MatrixXd &upsilon, const MatrixXd &predictCov);
    MatrixXd PUpdate(const MatrixXd &gain, const MatrixXd &pPred, const MatrixXd &pMeas);
    MatrixXd xUpdate(const VectorXd &xPred, const MatrixXd &gain, const VectorXd &xMeas, const VectorXd &xMeasPred);



signals:

};

#endif // KALMANFILTER_H
