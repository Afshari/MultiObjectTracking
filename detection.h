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
    explicit Detection(QObject *parent = nullptr);

    MeasurementModel measurementModel;
    VectorXd x;

signals:

};

#endif // DETECTION_H
