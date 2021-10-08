#ifndef QTEST_SENSOR_H
#define QTEST_SENSOR_H

#include <QObject>
#include <iostream>

#include "inc/sensor.h"

class QTestSensor : public QObject
{
    Q_OBJECT
public:
    explicit QTestSensor(QObject *parent = nullptr);

private slots:
    void testConstructor();

};

#endif // QTEST_SENSOR_H
