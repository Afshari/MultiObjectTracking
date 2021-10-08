#ifndef DETECTION_H
#define DETECTION_H

#include <QObject>
#include <Eigen/Dense>
#include "measurementmodel.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Detection : public QObject
{
    Q_OBJECT
public:
    enum DetectionType { miss, detect };

    explicit Detection(Detection::DetectionType type, QObject *parent = nullptr);

    MeasurementModel measurementModel;
    VectorXd *x;


    DetectionType type;

signals:

};

#endif // DETECTION_H
