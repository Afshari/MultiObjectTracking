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


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                          QObject *parent = nullptr);

    void predict(const State &prior, int dt);
    State* update(const State &state, const MeasurementModel &measurementModel,
                 const VectorXd &measurement, const MeasurementPrediction &measurementPrediction);

    friend class TestKalmanFilter;

private:
    VectorXd prior;
    VectorXd xPred;
    VectorXd pPred;
    VectorXd xPosterior;
    VectorXd pPosterior;

    MeasurementModel *measurementModel;
    TransitionModel  *transitionModel;

    VectorXd xPredict(const State &prior, int dt);
    MatrixXd PPredict(const State &prior, int dt);

    MatrixXd kalmanGain(const MatrixXd &crossCov, const MatrixXd &predictCov);
    MatrixXd PUpdate(const MatrixXd &gain, const MatrixXd &pPred, const MatrixXd &pMeas);
    MatrixXd xUpdate(const VectorXd &xPred, const MatrixXd &gain, const VectorXd &xMeas, const VectorXd &xMeasPred);
    MeasurementPrediction *predictMeasurement(State *predState);


signals:

};

#endif // KALMANFILTER_H
