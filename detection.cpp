#include "detection.h"

Detection::Detection(Detection::DetectionType type, QObject *parent) : QObject(parent) {

    this->type = type;
}
