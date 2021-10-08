#include "inc/qtest_measurement_constant_velocity.h"

QTestMeasurementConstantVelocity::QTestMeasurementConstantVelocity(QObject *parent) : QObject(parent) {

}

void QTestMeasurementConstantVelocity::testH() {

    MatrixXd H_ref(2, 4);
    H_ref <<  1,     0,     0,     0,
              0,     1,     0,     0;

    Vector4d x(1, 4, 5, 3);
    MeasurementConstantVelocity cvMeasurement(5);
    MatrixXd H = *cvMeasurement.H(x);

    QVERIFY(H.isApprox(H_ref, 1e-4));
}


void QTestMeasurementConstantVelocity::testh() {

    Vector2d h_ref(1, 4);

    VectorXd x = Vector4d(1, 4, 5, 3);

    MeasurementConstantVelocity cvMeasurement(5);
    MatrixXd h = *cvMeasurement.h(x);

    QVERIFY(h.isApprox(h_ref, 1e-4));
}


void QTestMeasurementConstantVelocity::testR() {

    MatrixXd R_ref(2, 2);
    R_ref << 25, 0,
             0,  25;

    MeasurementConstantVelocity cvMeasurement(5);
    MatrixXd R = *cvMeasurement.R;

    QVERIFY(R.isApprox(R_ref, 1e-4));
}





