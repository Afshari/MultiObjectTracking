#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QObject>
#include <QTest>
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "measurementmodel.h"
#include "transitionmodel.h"
#include "transitionlineargaussian.h"
#include "stategaussian.h"


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                          QObject *parent = nullptr);

    void predict(StateGaussian *prior, int dt);
    void update();

private:
    VectorXd prior;
    VectorXd xPred;
    VectorXd pPred;
    VectorXd xPosterior;
    VectorXd pPosterior;

    MeasurementModel *measurementModel;
    TransitionModel *transitionModel;

    VectorXd xPredict(StateGaussian *prior, int dt);

private slots:
    void testxPredictDt0();
    void testxPredictDt1();

signals:

};

#endif // KALMANFILTER_H
