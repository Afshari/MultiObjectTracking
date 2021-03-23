#ifndef MEASUREMENTPREDICTION_H
#define MEASUREMENTPREDICTION_H

#include <QObject>
#include <Eigen/Dense>
#include "state.h"
#include "measurementmodel.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementPrediction : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementPrediction(State *state, VectorXd *predMeas, MatrixXd *innovationCov,
                MatrixXd *crossCov, QObject *parent = nullptr);

    State *state;
    VectorXd *predMeas;
    MatrixXd *innovationCov;
    MatrixXd *crossCov;


private:

signals:

};

#endif // MEASUREMENTPREDICTION_H
