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
#include "stategaussian.h"


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                          QObject *parent = nullptr);

    void predict(const StateGaussian &prior, int dt);
    void update();

    friend class TestKalmanFilter;

private:
    VectorXd prior;
    VectorXd xPred;
    VectorXd pPred;
    VectorXd xPosterior;
    VectorXd pPosterior;

    MeasurementModel *measurementModel;
    TransitionModel *transitionModel;

    VectorXd xPredict(const StateGaussian &prior, int dt);
    MatrixXd PPredict(const StateGaussian &prior, int dt);

    MatrixXd kalmanGain(const MatrixXd &crossCov, const MatrixXd &predictCov);
    MatrixXd PUpdate(const MatrixXd &gain, const MatrixXd &pPred, const MatrixXd &pMeas);
    MatrixXd xUpdate(const VectorXd &xPred, const MatrixXd &gain, const VectorXd &xMeas, const VectorXd &xMeasPred);
    MeasurementPrediction *predictMeasurement(StateGaussian *predState);


signals:

};

#endif // KALMANFILTER_H
