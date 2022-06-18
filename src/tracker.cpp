#include "inc/tracker.h"

Tracker::Tracker(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state, shared_ptr<Sensor> sensor,
                 double gating_size, double reduction_M, double w_min, QObject *parent) : QObject(parent) {

    this->estimator = estimator;
    this->sensor = sensor;
    this->gating_size = gating_size;
    this->w_min = log(w_min);
    this->state = initial_state;
    this->reduction_M = reduction_M;
}

VectorXd Tracker::getX() {
    return this->state->getX();
}
