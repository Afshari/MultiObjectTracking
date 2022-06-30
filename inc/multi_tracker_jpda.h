#ifndef MULTI_TRACKER_JPDA_H
#define MULTI_TRACKER_JPDA_H

#include <QObject>
#include <Eigen/Dense>
#include <set>

#include "inc/hypothesis.h"
#include "inc/multi_tracker.h"
#include "libs/murty/murty.h"

class MultiTrackerJPDA : public MultiTracker {
    Q_OBJECT
public:
    explicit MultiTrackerJPDA(shared_ptr<Estimator> estimator, PtrVecState states,
                              shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                              double w_min = 0, QObject *parent = nullptr);

    virtual void step(const MatrixXd &z, bool debug=false);
    virtual VectorXd getX(int idx);

};

#endif // MULTI_TRACKER_JPDA_H
