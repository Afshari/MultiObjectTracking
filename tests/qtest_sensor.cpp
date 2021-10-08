#include "tests/inc/qtest_sensor.h"

QTestSensor::QTestSensor(QObject *parent) : QObject(parent) {

}


void QTestSensor::testConstructor() {

    double P_D = 0.7;
    double lambda_c = 60;

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    Sensor sensor(P_D, lambda_c, range_c);

    QVERIFY( P_D == sensor.get_P_D() );
    QVERIFY( lambda_c == sensor.get_lambda_c() );
    QVERIFY( range_c.isApprox(sensor.get_range_c(), 1e-4) );
    QVERIFY( abs( sensor.get_pdf_c() - 7.95774e-5  ) < 1e-7 );
    QVERIFY( abs( sensor.get_intensity() - 0.00477  ) < 1e-4 );
}

