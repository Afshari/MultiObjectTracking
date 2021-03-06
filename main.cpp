
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
#include "Tests/testutils.h"

#include "debugserver.h"
#include "tcpserver.h"

#include <iostream>

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define RUN_TEST    1
#define RUN_DEBUG   2
#define RUN_APP     3

#define RUN_STATUS  RUN_APP

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

#if RUN_STATUS == RUN_APP

    TCPServer tcpServer;
    tcpServer.start();

#elif RUN_STATUS == RUN_DEBUG

    DebugServer server;
    server.start();

#elif RUN_STATUS == RUN_TEST

    TestMeasurementLinearGaussian measurementLinearGaussian;
    QTest::qExec(&measurementLinearGaussian);

    TestPDA testPDA;
    QTest::qExec(&testPDA);

    TestMultiHypothesis testMultiHypothesis;
    QTest::qExec(&testMultiHypothesis);

    TestKalmanFilter kalmanFilter;
    QTest::qExec(&kalmanFilter);

    TestUtils utils;
    QTest::qExec(&utils);

#endif

    return a.exec();
}
