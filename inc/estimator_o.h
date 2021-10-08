#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <QObject>
#include <Eigen/Dense>

#include "state.h"
#include "measurementmodel.h"
#include "inc/transition_model.h"
#include "measurementprediction.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class Estimator : public QObject
{
    Q_OBJECT
public:
    explicit Estimator(MeasurementModel *measurementModel, TransitionModel *transitionModel, QObject *parent = nullptr);

    MeasurementModel *measurementModel;
    TransitionModel  *transitionModel;



public:
    virtual MeasurementPrediction *predictMeasurement(State *predState) { std::ignore = predState; return nullptr; };
    virtual State* predict(const State &prior, int dt) { std::ignore = prior; std::ignore = dt; return nullptr; };
    virtual State* update(const State &state, const MeasurementModel &measurementModel,
                          const VectorXd &measurement,
                          const MeasurementPrediction &measurementPrediction)
                  { std::ignore = state; std::ignore = measurementModel; std::ignore = measurement;
                    std::ignore = measurementPrediction; return  nullptr; };


signals:

};

#endif // ESTIMATOR_H
