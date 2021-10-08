#include "inc/transition_constant_velocity.h"

TransitionConstantVelocity::TransitionConstantVelocity(float T, float sigma, QObject *parent) :
    TransitionModel(T, parent) {

    this->d = 4;
    this->sigma = sigma;

    MatrixXd _Q(4, 4);
    _Q <<   pow(T, 4)/4,    0,              pow(T, 3)/2,    0,
            0,              pow(T, 4)/4,    0,               pow(T, 3)/2,
            pow(T, 3)/2,    0,              pow(T, 2),      0,
            0,              pow(T, 3)/2,    0,              pow(T, 2);
    _Q = _Q * pow(this->sigma, 2);
    this->_Q = make_shared<MatrixXd>( _Q );
}


shared_ptr<MatrixXd> TransitionConstantVelocity::Q() {

    return _Q;
}


shared_ptr<MatrixXd> TransitionConstantVelocity::F(const VectorXd &x) {

    std::ignore = x;

    MatrixXd _F(4, 4);
    float T = this->T;
    _F <<   1,  0,  T,  0,
            0,  1,  0,  T,
            0,  0,  1,  0,
            0,  0,  0,  1;
    return make_shared<MatrixXd>( _F );
}


shared_ptr<VectorXd> TransitionConstantVelocity::f(const VectorXd &x) {

    return make_shared<VectorXd>( (*this->F(x)) * x );
}


