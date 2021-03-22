
#include <QCoreApplication>
#include <QTest>
#include "transitionmodel.h"
#include "transitionlineargaussian.h"
#include "measurementlineargaussian.h"
#include "stategaussian.h"
#include "kalmanfilter.h"

#include "Tests/testkalmanfilter.h"
#include "Tests/testmeasurementlineargaussian.h"
#include "Tests/testpda.h"
#include "Tests/testmultihypothesis.h"

#include <iostream>

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    TestMeasurementLinearGaussian measurementLinearGaussian;
    QTest::qExec(&measurementLinearGaussian);

    TestKalmanFilter kalmanFilter;
    QTest::qExec(&kalmanFilter);

    TestPDA testPDA;
    QTest::qExec(&testPDA);


    TestMultiHypothesis testMultiHypothesis;
    QTest::qExec(&testMultiHypothesis);

    return a.exec();
}
