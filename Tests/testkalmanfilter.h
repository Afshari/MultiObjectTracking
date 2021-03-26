#ifndef TESTKALMANFILTER_H
#define TESTKALMANFILTER_H

#include <QObject>
#include <QTest>

#include "measurementlineargaussian.h"
#include "kalmanfilter.h"
#include "stategaussian.h"

class TestKalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit TestKalmanFilter(QObject *parent = nullptr);

signals:

private:
    KalmanFilter *kalman;

private slots:
    void initTestCase();

    void testxPredictDt0();
    void testxPredictDt1();

    void testPPredictDt0();
    void testPPredictDt1();

    void testK1();
    void testK2();

    void testPUpdate1();
    void testPUpdate2();

    void testxUpdate1();
    void testxUpdate2();

    void testPredictMeasurement1();

    void testUpdate1();

};


#endif // TESTKALMANFILTER_H
