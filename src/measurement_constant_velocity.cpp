#include "inc/measurement_constant_velocity.h"

MeasurementConstantVelocity::MeasurementConstantVelocity(float sigma, QObject *parent) :
    MeasurementModel(parent) {

    this->sigma = sigma;
    this->d = 2;

    MatrixXd _H(2, 4);
    _H <<   1, 0, 0, 0,
            0, 1, 0, 0;
    this->_H = make_shared<MatrixXd>( _H );

    MatrixXd _R(2, 2);
    _R <<   pow(this->sigma, 2),        0,
            0,                          pow(this->sigma, 2);
    this->R = make_shared<MatrixXd>( _R );
}


shared_ptr<MatrixXd> MeasurementConstantVelocity::H(const VectorXd &x) {

    std::ignore = x;
    return this->_H;
}



shared_ptr<VectorXd> MeasurementConstantVelocity::h(const VectorXd &x) {

    MatrixXd _h = (*this->H(x)) * x;
    return make_shared<VectorXd>( _h );
}
