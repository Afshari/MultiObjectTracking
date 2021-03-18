#ifndef TRANSITIONLINEARGAUSSIAN_H
#define TRANSITIONLINEARGAUSSIAN_H

#include "transitionmodel.h"
#include <iostream>
#include "utils.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;

class TransitionLinearGaussian : public TransitionModel
{
public:
    TransitionLinearGaussian(float q);

private:
    float q;
    MatrixXd constantVelocity;
    MatrixXd Q;


};

#endif // TRANSITIONLINEARGAUSSIAN_H
