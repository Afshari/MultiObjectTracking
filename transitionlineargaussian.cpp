#include "transitionlineargaussian.h"


TransitionLinearGaussian::TransitionLinearGaussian(float q) : constantVelocity(2, 2), Q(2, 2) {

    this->q = q;

    // define Q, with dt=1
    Q << 0.33, 0.5,
         0.5,  1;
    Q = this->q * Q;

//    std::cout << Q << std::endl;

    constantVelocity << 1, 1,
                        0, 1;

//    std::cout << constantVelocity << std::endl;
//    std::cout << Utils::blkdiag(constantVelocity, 2) << std::endl;

}


MatrixXd TransitionLinearGaussian::transitionFunction(int dt) {
    if(dt == 0)
        return MatrixXd::Identity(4, 4);
    else
        return Utils::blkdiag(constantVelocity, 2);
}
