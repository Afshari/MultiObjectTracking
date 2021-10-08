#ifndef TRANSITION_2D_TURN_H
#define TRANSITION_2D_TURN_H

#include <QObject>
#include <cmath>

#include "inc/transition_model.h"

class Transition2dTurn : public TransitionModel {
    Q_OBJECT
public:
    explicit Transition2dTurn(float T, float sigma_v, float sigma_omega, QObject *parent = nullptr);

    virtual shared_ptr<MatrixXd> F(const VectorXd &x);
    virtual shared_ptr<VectorXd> f(const VectorXd &x);
    virtual shared_ptr<MatrixXd> Q();


private:
    float sigma_v;
    float sigma_omega;
    float d;
    shared_ptr<MatrixXd> _Q;

};

#endif // TRANSITION_2D_TURN_H
