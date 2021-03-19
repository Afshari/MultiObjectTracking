#include "transitionlineargaussian.h"


TransitionLinearGaussian::TransitionLinearGaussian(float q) : constantVelocity(2, 2), Q(2, 2) {

    this->q = q;

    Q << 0.33, 0.5,
         0.5,  1;

    constantVelocity << 1, 1,
                        0, 1;

}

MatrixXd TransitionLinearGaussian::transitionCov(int dt) {
    if(dt == 0)
        return MatrixXd::Zero(4, 4);
    else
        return Utils::blkdiag(q * Q, 2);
}


MatrixXd TransitionLinearGaussian::transitionFunction(int dt) {
    if(dt == 0)
        return MatrixXd::Identity(4, 4);
    else
        return Utils::blkdiag(constantVelocity, 2);
}
