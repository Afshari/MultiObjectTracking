#include "testmeasurementlineargaussian.h"

TestMeasurementLinearGaussian::TestMeasurementLinearGaussian(QObject *parent) : QObject(parent) {

}


void TestMeasurementLinearGaussian::initTestCase() {

    MatrixXd *measurementNoiseCovariance = new MatrixXd(2, 2);
    *measurementNoiseCovariance <<    0.75,  0,
                                      0,     0.75;

    measure = new MeasurementLinearGaussian(measurementNoiseCovariance);
}


void TestMeasurementLinearGaussian::testInnovationCov() {

    MatrixXd ref(2, 2);
    ref << 1.92522955, 0.03433435,
           0.03433435, 1.92480954;


    MatrixXd measCrossCov(4, 2);
    measCrossCov << 1.17522955, 0.03433435,
                    0.5025,     0.,
                    0.03433435, 1.17480954,
                    0.,         0.5025;

    QVERIFY2(measure->innovationCov(measCrossCov).isApprox(ref, 1e-4), "Innovation Covariance Not Correct");
}

void TestMeasurementLinearGaussian::testCrossCov() {

    MatrixXd ref(4, 2);
    ref << 1.17522955, 0.03433435,
           0.5025,     0.,
           0.03433435, 1.17480954,
           0.,         0.5025;


    MatrixXd predCov(4, 4);
    predCov << 1.17522955, 0.5025,     0.03433435, 0.,
                    0.5025,     0.505,      0.,         0.,
                    0.03433435, 0.,         1.17480954, 0.5025,
                    0.,         0.,         0.5025,     0.505;

    QVERIFY2(measure->crossCov(predCov).isApprox(ref, 1e-4), "Cross Covariance Not Correct");
}
