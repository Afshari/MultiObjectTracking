
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

#include "debugserver.h"

#include <iostream>

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    DebugServer server;
    server.start();

//    TestMeasurementLinearGaussian measurementLinearGaussian;
//    QTest::qExec(&measurementLinearGaussian);

//    TestPDA testPDA;
//    QTest::qExec(&testPDA);

//    TestMultiHypothesis testMultiHypothesis;
//    QTest::qExec(&testMultiHypothesis);

//    TestKalmanFilter kalmanFilter;
//    QTest::qExec(&kalmanFilter);

    return a.exec();
}
