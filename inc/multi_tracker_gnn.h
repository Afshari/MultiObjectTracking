#ifndef MULTI_TRACKER_GNN_H
#define MULTI_TRACKER_GNN_H

#include <QObject>
#include <Eigen/Dense>
#include <set>

#include "inc/multi_tracker.h"
#include "inc/hypothesis.h"
#include "libs/hungarian/hungarian.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MultiTrackerGNN : public MultiTracker {
    Q_OBJECT
public:
    explicit MultiTrackerGNN(shared_ptr<Estimator> estimator, PtrVecState states,
                        shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                        double w_min = 0, QObject *parent = nullptr);
    virtual void step(const MatrixXd &z);

};

#endif // MULTI_TRACKER_GNN_H
