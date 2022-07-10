#ifndef TRACKER_H
#define TRACKER_H

#include <QObject>
#include <Eigen/Dense>


#include "inc/transition_model.h"
#include "inc/measurement_model.h"
#include "inc/sensor.h"
#include "inc/estimator.h"

using std::shared_ptr;
using std::make_shared;


class Tracker : public QObject {
    Q_OBJECT
public:
    explicit Tracker(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state, shared_ptr<Sensor> sensor,
                     double gating_size, double reduction_M, double w_min = 0, QObject *parent = nullptr);
    virtual void step(const MatrixXd &z) { std::ignore = z; }
    virtual VectorXd getX();
    virtual VectorXd getUpdatedX();

protected:
    shared_ptr<Estimator> estimator;
    shared_ptr<Sensor>  sensor;
    shared_ptr<State>   state;
    VectorXd updated_x;


    double gating_size;
    double w_min;
    double reduction_M;

};

#endif // TRACKER_H
