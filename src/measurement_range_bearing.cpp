#include "inc/measurement_range_bearing.h"

MeasurementRangeBearing::MeasurementRangeBearing(float sigma_r, float sigma_b, shared_ptr<Vector2d> s,
                                                 QObject *parent) : MeasurementModel(parent) {


    this->sigma_r = sigma_r;
    this->sigma_b = sigma_b;
    this->s = s;

    MatrixXd _R(2, 2);
    _R <<   pow(this->sigma_r, 2),      0,
            0,                          pow(this->sigma_b, 2);
    this->R = make_shared<MatrixXd>( _R );
}

shared_ptr<MatrixXd> MeasurementRangeBearing::H(const VectorXd &x) {


    Vector2d s = *this->s;

    MatrixXd _H = MatrixXd::Zero(2, x.rows());
    _H(0, 0) =  (x[0] - s[0]) / _range(x);
    _H(0, 1) =  (x[1] - s[1]) / _range(x);
    _H(1, 0) = -(x[1] - s[1]) / pow(_range(x), 2 );
    _H(1, 1) =  (x[0] - s[0]) / pow(_range(x), 2 );

    return make_shared<MatrixXd>( _H );
}

shared_ptr<VectorXd> MeasurementRangeBearing::h(const VectorXd & x) {

    return make_shared<VectorXd>( Vector2d( _range(x), _bearing(x) ) );
}


float MeasurementRangeBearing::_range(const VectorXd & x) {

    Vector2d s = *this->s;

    return ( x.block(0, 0, 2, 1) - s ).norm();
}


float MeasurementRangeBearing::_bearing(const VectorXd & x) {

    Vector2d s = *this->s;

    return atan2( x[1] - s[1], x[0] - s[0] );
}



