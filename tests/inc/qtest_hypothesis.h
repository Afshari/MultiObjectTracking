#ifndef QTESTHYPOTHESIS_H
#define QTESTHYPOTHESIS_H

#include <QObject>
#include <QTest>
#include <Eigen/Dense>
#include "inc/hypothesis.h"

using Eigen::Matrix2Xd;
using Eigen::VectorXd;


class QTestHypothesis : public QObject {
    Q_OBJECT
public:
    explicit QTestHypothesis(QObject *parent = nullptr);

private slots:
    void testNormalizeLogWeights_1();
    void testNormalizeLogWeights_2();
    void testNormalizeLogWeights_3();
    void testNormalizeLogWeights_4();


};

#endif // QTESTHYPOTHESIS_H
