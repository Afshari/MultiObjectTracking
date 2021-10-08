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
    explicit MeasurementPrediction(State *statePred, VectorXd* xOfZPred, VectorXd *zPred, MatrixXd *S,
                MatrixXd *upsilon, QObject *parent = nullptr);

    State       *statePred;
    VectorXd    *xOfZPred;
    VectorXd    *zPred;
    MatrixXd    *S;
    MatrixXd    *upsilon;


private:

signals:

};

#endif // MEASUREMENTPREDICTION_H
