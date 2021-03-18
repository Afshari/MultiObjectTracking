#ifndef TRANSITIONLINEARGAUSSIAN_H
#define TRANSITIONLINEARGAUSSIAN_H

#include "transitionmodel.h"
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class TransitionLinearGaussian : public TransitionModel
{
public:
    TransitionLinearGaussian();

private:
    MatrixXd constantVelocity;


};

#endif // TRANSITIONLINEARGAUSSIAN_H
