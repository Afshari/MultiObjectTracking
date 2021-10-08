#ifndef MEASUREMENT_CONSTANT_VELOCITY_H
#define MEASUREMENT_CONSTANT_VELOCITY_H

#include <QObject>

#include "inc/measurement_model.h"

class MeasurementConstantVelocity : public MeasurementModel {
    Q_OBJECT
public:
    explicit MeasurementConstantVelocity(float sigma, QObject *parent = nullptr);
    virtual shared_ptr<MatrixXd> H(const VectorXd &x);
    virtual shared_ptr<VectorXd> h(const VectorXd &state);

protected:
    float sigma;

#if RUN_TYPE == RUN_TEST
    friend class QTestMeasurementConstantVelocity;
#endif

};

#endif // MEASUREMENT_CONSTANT_VELOCITY_H
