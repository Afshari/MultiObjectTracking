#ifndef STATEGAUSSIAN_H
#define STATEGAUSSIAN_H

#include "state.h"

class StateGaussian : public State
{
public:
    StateGaussian(VectorXd *x, MatrixXd *p);
};

#endif // STATEGAUSSIAN_H
