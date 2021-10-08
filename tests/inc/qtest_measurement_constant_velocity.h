#ifndef QTEST_MEASUREMENT_CONSTANT_VELOCITY_H
#define QTEST_MEASUREMENT_CONSTANT_VELOCITY_H

#include <QObject>
#include <QtTest/QtTest>
#include <iostream>
#include <Eigen/Dense>

#include "inc/measurement_constant_velocity.h"

using Eigen::Vector2d;
using Eigen::Vector4d;

class QTestMeasurementConstantVelocity : public QObject {
    Q_OBJECT
public:
    explicit QTestMeasurementConstantVelocity(QObject *parent = nullptr);

private slots:
    void testH();
    void testh();
    void testR();


};

#endif // QTEST_MEASUREMENT_CONSTANT_VELOCITY_H
