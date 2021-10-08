#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <QObject>
#include <QTest>
#include <Eigen/Dense>
#include "inc/utils.h"

using Eigen::Matrix2Xd;
using Eigen::VectorXd;

class QTestUtils : public QObject {
    Q_OBJECT
public:
    explicit QTestUtils(QObject *parent = nullptr);

private slots:
    void testMean();
    void testCovar();
    void testCovar2();


};

#endif // TEST_UTILS_H
