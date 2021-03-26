#include "estimator.h"



Estimator::Estimator(MeasurementModel *measurementModel, TransitionModel  *transitionModel, QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;

}
