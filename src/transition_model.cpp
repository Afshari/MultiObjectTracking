#include "inc/transition_model.h"


TransitionModel::TransitionModel(float T, QObject *parent) : QObject(parent) {

    this->T = T;
}

