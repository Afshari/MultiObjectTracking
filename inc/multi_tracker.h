#ifndef MULTI_TRACKER_H
#define MULTI_TRACKER_H

#include <QObject>

#include "inc/sensor.h"
#include "inc/estimator.h"
#include "inc/state.h"

using std::shared_ptr;
using std::make_shared;

class MultiTracker : public QObject {
    Q_OBJECT
public:
    explicit MultiTracker(shared_ptr<Estimator> estimator, PtrVecState states, shared_ptr<Sensor> sensor,
                          double gating_size, int reduction_M, double w_min = 0, QObject *parent = nullptr);
    virtual void step(const MatrixXd &z)  { std::ignore = z; }

protected:
    shared_ptr<Estimator>   estimator;
    shared_ptr<Sensor>      sensor;
    PtrVecState             states;


    double gating_size;
    double w_min;
    int reduction_M;


};

#endif // MULTI_TRACKER_H
