#include "measurementprediction.h"

MeasurementPrediction::MeasurementPrediction(State *predState, VectorXd *predMeas, MatrixXd *innovationCov,
                                             MatrixXd *crossCov,QObject *parent) : QObject(parent) {

    this->state = predState;
    this->predMeas = predMeas;
    this->innovationCov = innovationCov;
    this->crossCov = crossCov;
}
