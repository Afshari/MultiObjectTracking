#include "measurementprediction.h"

MeasurementPrediction::MeasurementPrediction(State *statePred, VectorXd* xzPred, VectorXd *zPred, MatrixXd *S,
                                             MatrixXd *upsilon,QObject *parent) : QObject(parent) {

    this->statePred = statePred;
    this->xOfZPred = xzPred;
    this->zPred = zPred;
    this->S = S;
    this->upsilon = upsilon;
}
