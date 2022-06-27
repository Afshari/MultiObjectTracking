#ifndef _MULTI_TRACKER_H_
#define _MULTI_TRACKER_H_

#include <QObject>

#include "inc/sensor.h"
#include "inc/estimator.h"
#include "inc/state.h"

using std::shared_ptr;
using std::make_shared;

class MultiTracker: public QObject {
    Q_OBJECT
public:
    explicit MultiTracker(shared_ptr<Estimator> estimator, PtrVecState states, shared_ptr<Sensor> sensor,
                          double gating_size, int reduction_M, double w_min = 0, QObject *parent = nullptr);
    virtual void step(const MatrixXd &z)  { std::ignore = z; }
    virtual VectorXd getX(int idx) { return VectorXd(); }

    void enablePrintResult() { print_result = true; }

protected:
    shared_ptr<Estimator>   estimator;
    shared_ptr<Sensor>      sensor;
    PtrVecState             states;


    double gating_size;
    double w_min;
    int reduction_M;
    bool print_result;


};

#endif // _MULTI_TRACKER_H_
