#include "inc/multi_tracker.h"

MultiTracker::MultiTracker(shared_ptr<Estimator> estimator, PtrVecState states, shared_ptr<Sensor> sensor,
                           double gating_size, int reduction_M, double w_min, QObject *parent) : QObject(parent) {

    this->estimator     = estimator;
    this->states        = states;
    this->sensor        = sensor;
    this->gating_size   = gating_size;
    this->reduction_M   = reduction_M;
    this->w_min         = log(w_min);
}
