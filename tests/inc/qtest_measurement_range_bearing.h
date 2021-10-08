#ifndef QTEST_MEASUREMENT_RANGE_BEARING_H
#define QTEST_MEASUREMENT_RANGE_BEARING_H

#include <QObject>
#include <QtTest/QtTest>
#include <iostream>
#include <Eigen/Dense>

#include "inc/measurement_range_bearing.h"

using Eigen::Vector2d;
using Eigen::Vector4d;
using std::shared_ptr;
using std::make_shared;


class QTestMeasurementRangeBearing : public QObject {
    Q_OBJECT
public:
    explicit QTestMeasurementRangeBearing(QObject *parent = nullptr);

private slots:
    void testRange();
    void testBearing();
    void testh();
    void testH();
    void testR();


};

#endif // QTEST_MEASUREMENT_RANGE_BEARING_H
