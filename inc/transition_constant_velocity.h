#ifndef TRANSITION_CONSTANT_VELOCITY_H
#define TRANSITION_CONSTANT_VELOCITY_H

#include <QObject>
#include <cmath>
#include "inc/transition_model.h"

class TransitionConstantVelocity : public TransitionModel {
    Q_OBJECT
public:
    explicit TransitionConstantVelocity(float T, float sigma, QObject *parent = nullptr);

    virtual shared_ptr<MatrixXd> F(const VectorXd &x);
    virtual shared_ptr<VectorXd> f(const VectorXd &x);
    virtual shared_ptr<MatrixXd> Q();

protected:
    int d;
    float sigma;

signals:

};

#endif // TRANSITION_CONSTANT_VELOCITY_H
