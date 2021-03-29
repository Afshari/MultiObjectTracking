#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <QObject>
#include <QTest>
#include <Eigen/Dense>
#include "utils.h"

using Eigen::Matrix2Xd;
using Eigen::VectorXd;

class TestUtils : public QObject
{
    Q_OBJECT
public:
    explicit TestUtils(QObject *parent = nullptr);

private slots:
    void testMean();
    void testCovar();
    void testCovar2();

signals:

};

#endif // TESTUTILS_H
