
#include <QCoreApplication>
#include <QTest>
#include "transitionmodel.h"
#include "transitionlineargaussian.h"
#include "measurementlineargaussian.h"
#include "stategaussian.h"
#include "kalmanfilter.h"

#include <iostream>

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

//    VectorXd x(4);
//    x << -0.16260711, 1, 0.12305334, 1;
//    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
//    MatrixXd p = pVector.asDiagonal();

//    StateGaussian state(&x, &p);

    MeasurementLinearGaussian measure;
    TransitionLinearGaussian transitionLinearGaussian(0.005);
    KalmanFilter kalman(&measure, &transitionLinearGaussian);

    QTest::qExec(&kalman);

//    kalman.predict(&state);

    return a.exec();
}
