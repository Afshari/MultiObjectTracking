#include "transitionlineargaussian.h"

TransitionLinearGaussian::TransitionLinearGaussian() : constantVelocity(4, 4) {

    constantVelocity << 1, 1, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 1,
                        0, 0, 0, 1;

    std::cout << constantVelocity << std::endl;

}
