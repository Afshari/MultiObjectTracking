#ifndef TRANSITION_LINEAR_GAUSSIAN_H
#define TRANSITION_LINEAR_GAUSSIAN_H


#include <iostream>
#include "utils.h"
#include <Eigen/Dense>

#include "inc/transition_model.h"

using Eigen::MatrixXd;

class TransitionLinearGaussian : public TransitionModel
{
public:
    TransitionLinearGaussian(float q);
    virtual MatrixXd transitionFunction(int dt) override;
    virtual MatrixXd transitionCov(int dt) override;


protected:



private:
    float q;
    MatrixXd constantVelocity;
    MatrixXd Q;


};

#endif // TRANSITION_LINEAR_GAUSSIAN_H
