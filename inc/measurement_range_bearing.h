#ifndef MEASUREMENT_RANGE_BEARING_H
#define MEASUREMENT_RANGE_BEARING_H

#include <QObject>
#include <Eigen/Dense>

#include "inc/measurement_model.h"

using Eigen::Vector2d;
using Eigen::Vector4d;
using std::shared_ptr;
using std::make_shared;

class MeasurementRangeBearing : public MeasurementModel {
    Q_OBJECT
public:
    explicit MeasurementRangeBearing(float sigma_r, float sigma_b, shared_ptr<Vector2d> s , QObject *parent = nullptr);
    virtual shared_ptr<MatrixXd> H(const VectorXd &x);
    virtual shared_ptr<VectorXd> h(const VectorXd &x);

protected:
    float sigma_r;
    float sigma_b;
    shared_ptr<Vector2d> s;

    float _range(const VectorXd &x);
    float _bearing(const VectorXd &x);

#if RUN_TYPE == RUN_TEST

    friend class QTestMeasurementRangeBearing;

#endif


};

#endif // MEASUREMENT_RANGE_BEARING_H
