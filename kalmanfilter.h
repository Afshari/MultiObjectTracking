#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QObject>
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "measurementmodel.h"
#include "transitionmodel.h"


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                          QObject *parent = nullptr);

    void predict();
    void update();

private:
    VectorXd prior;
    VectorXd xPred;
    VectorXd pPred;
    VectorXd xPosterior;
    VectorXd pPosterior;

    MeasurementModel *measurementModel;
    TransitionModel *transitionModel;

signals:

};

#endif // KALMANFILTER_H
