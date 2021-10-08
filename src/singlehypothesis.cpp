#include "inc/singlehypothesis.h"

SingleHypothesis::SingleHypothesis(Detection *detection, State *state, TransitionModel *transitionModel,
                                   MeasurementPrediction *measurementPrediction, double probability,
                                   QObject *parent) : QObject(parent) {

    this->detection = detection;
    this->state = state;
    this->transitionModel = transitionModel;
    this->measurementPrediction = measurementPrediction;
    this->probability = probability;
}
