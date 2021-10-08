#ifndef QTEST_TRANSITION_CONSTANT_VELOCITY_H
#define QTEST_TRANSITION_CONSTANT_VELOCITY_H

#include <QObject>
#include <QtTest/QtTest>
#include <iostream>
#include <Eigen/Dense>

#include "inc/transition_constant_velocity.h"

using Eigen::MatrixXd;
using Eigen::Vector4d;

class QTestTransitionConstantVelocity : public QObject {
    Q_OBJECT
public:
    explicit QTestTransitionConstantVelocity(QObject *parent = nullptr);

private slots:
    void testQ();
    void testF();
    void testf();


};

#endif // QTEST_TRANSITION_CONSTANT_VELOCITY_H
