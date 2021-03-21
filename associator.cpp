#include "associator.h"

Associator::Associator(float spatialClutterIntensity, float P_D, float P_G, QObject *parent) : QObject(parent) {

    this->spatialClutterIntensity = spatialClutterIntensity;
    this->P_D = P_D;
    this->P_G = P_G;

}
