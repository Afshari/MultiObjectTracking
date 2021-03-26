#include "associator.h"

Associator::Associator(Estimator *estimator, float spatialClutterIntensity, float P_D, float P_G, QObject *parent) : QObject(parent) {

    this->estimator = estimator;
    this->spatialClutterIntensity = spatialClutterIntensity;
    this->P_D = P_D;
    this->P_G = P_G;

}
