#ifndef TESTMEASUREMENTLINEARGAUSSIAN_H
#define TESTMEASUREMENTLINEARGAUSSIAN_H

#include <QObject>
#include <QTest>

#include "measurementlineargaussian.h"

class TestMeasurementLinearGaussian : public QObject
{
    Q_OBJECT
public:
    explicit TestMeasurementLinearGaussian(QObject *parent = nullptr);

private:
    MeasurementLinearGaussian *measure;

private slots:
    void initTestCase();
    void testInnovationCov();
    void testCrossCov();


signals:

};

#endif // TESTMEASUREMENTLINEARGAUSSIAN_H
