#ifndef TESTMEASUREMENTLINEARGAUSSIAN_H
#define TESTMEASUREMENTLINEARGAUSSIAN_H

#include <QObject>
#include <QTest>

#include "inc/measurementlineargaussian.h"

class TestMeasurementLinearGaussian : public QObject
{
    Q_OBJECT
public:
    explicit TestMeasurementLinearGaussian(QObject *parent = nullptr);

private:
    MeasurementLinearGaussian *measure;

private slots:
    void initTestCase();
    void testS();
    void testUpsilon();
    void testh();


signals:

};

#endif // TESTMEASUREMENTLINEARGAUSSIAN_H
