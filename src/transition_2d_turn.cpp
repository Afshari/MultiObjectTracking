#include "inc/transition_2d_turn.h"

Transition2dTurn::Transition2dTurn(float T, float sigma_v, float sigma_omega, QObject *parent) :
    TransitionModel(T, parent) {

    this->sigma_v = sigma_v;
    this->sigma_omega = sigma_omega;
    this->d = 5;

    MatrixXd G(5, 2);
    G <<    0,      0,
            0,      0,
            1,      0,
            0,      0,
            0,      1;
    MatrixXd Q(2, 2);
    Q <<    pow(sigma_v, 2),    0,
            0,                  pow(sigma_omega, 2);
    Q = G * Q * G.transpose();
    this->_Q = make_shared<MatrixXd>( Q );
}


shared_ptr<MatrixXd> Transition2dTurn::F(const VectorXd &x) {

    MatrixXd _F(5, 5);
    float T = this->T;
    _F <<   1,      0,      T*cos(x[3]),     -T*x[2]*sin(x[3]),     0,
            0,      1,      T*sin(x[3]),      T*x[2]*cos(x[3]),     0,
            0,      0,      1,                0,                    0,
            0,      0,      0,                1,                    T,
            0,      0,      0,                0,                    1;
    return make_shared<MatrixXd>( _F );
}

shared_ptr<VectorXd> Transition2dTurn::f(const VectorXd &x) {

    float T = this->T;

    VectorXd result(5);
    result <<   T * x[2] * cos(x[3]),
                T * x[2] * sin(x[3]),
                0,
                T * x[4],
                0;
    return make_shared<VectorXd>( x + result );
}

shared_ptr<MatrixXd> Transition2dTurn::Q() {

    return this->_Q;
}






