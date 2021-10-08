#ifndef SINGLEHYPOTHESIS_H
#define SINGLEHYPOTHESIS_H

#include <QObject>
#include <Eigen/Dense>

#include "state.h"
#include "detection.h"
#include "measurementprediction.h"
#include "inc/transition_model.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

class SingleHypothesis : public QObject
{
    Q_OBJECT
public:
    explicit SingleHypothesis(Detection *detection, State *state, TransitionModel *transitionModel,
        MeasurementPrediction *measurementPrediction, double probability,  QObject *parent = nullptr);

    Detection *detection;
    State *state;
    TransitionModel *transitionModel;
    MeasurementPrediction *measurementPrediction;
    double probability;


signals:

};

#endif // SINGLEHYPOTHESIS_H
