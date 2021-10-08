#ifndef TRACKER_GAUSSIAN_SUM_H
#define TRACKER_GAUSSIAN_SUM_H

#include <QObject>
#include <Eigen/Dense>

#include "inc/tracker.h"
#include "inc/hypothesis.h"


class TrackerGaussianSum : public Tracker {
    Q_OBJECT
public:
    explicit TrackerGaussianSum(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                                shared_ptr<Sensor> sensor, double gating_size, double reduction_M,
                                double w_min = 0, QObject *parent = nullptr);
    virtual void step(const MatrixXd &z);

protected:
    PtrVecState             hypotheses;
    shared_ptr<VectorXd>    w_logs;

#if RUN_TYPE == RUN_DEBUG
    friend class DebugServer;
#endif

};

#endif // TRACKER_GAUSSIAN_SUM_H
