#include "tests/inc/qtest_measurement_range_bearing.h"

QTestMeasurementRangeBearing::QTestMeasurementRangeBearing(QObject *parent) : QObject(parent) {

}


void QTestMeasurementRangeBearing::testRange() {

    float _range_ref = 223.6068;

    shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d(300, 400) );
    Vector4d x( 100, 300, 20, 20);

    MeasurementRangeBearing rbMeasurement(5, M_PI/180, s);

    float _range = rbMeasurement._range( x );

    QVERIFY( abs(_range - _range_ref) < 1e-4 );
}


void QTestMeasurementRangeBearing::testBearing() {

    float _bearing_ref = -2.6779;

    shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d(300, 400) );
    Vector4d x( 100, 300, 20, 20 );

    MeasurementRangeBearing rbMeasurement(5, M_PI/180, s);

    float _bearing = rbMeasurement._bearing( x );

    QVERIFY( abs(_bearing - _bearing_ref) < 1e-4 );
}

void QTestMeasurementRangeBearing::testR() {

    MatrixXd R_ref(2, 2);
    R_ref << 25,        0,
             0,         0.0003;

    shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d(300, 400) );

    MeasurementRangeBearing rbMeasurement(5, M_PI/180, s);

    QVERIFY( rbMeasurement.R->isApprox(R_ref, 1e-4) );
}


void QTestMeasurementRangeBearing::testh() {

    Vector2d h_ref = Vector2d( 223.6068, -2.6779 );

    shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d(300, 400) );
    Vector4d x( 100, 300, 20, 20 );

    MeasurementRangeBearing rbMeasurement(5, M_PI/180, s);

    VectorXd h =  *rbMeasurement.h( x );

    QVERIFY( h.isApprox(h_ref, 1e-4) );
}


void QTestMeasurementRangeBearing::testH() {

    MatrixXd H_ref(2, 4);
    H_ref <<    -0.8944,   -0.4472,         0,         0,
                 0.0020,   -0.0040,         0,         0;

    shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d(300, 400) );
    Vector4d x( 100, 300, 20, 20 ) ;

    MeasurementRangeBearing rbMeasurement(5, M_PI/180, s);

    MatrixXd H =  *rbMeasurement.H( x );

    QVERIFY( H.isApprox(H_ref, 1e-4) );
}




