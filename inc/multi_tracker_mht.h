#ifndef MULTI_TRACKER_MHT_H
#define MULTI_TRACKER_MHT_H

#include <QObject>
#include <Eigen/Dense>
#include <set>

#include "inc/hypothesis.h"
#include "inc/multi_tracker.h"
#include "libs/murty/murty.h"


using Eigen::VectorXi;

class MultiTrackerMHT : public MultiTracker {
    Q_OBJECT
public:
    explicit MultiTrackerMHT(shared_ptr<Estimator> estimator, PtrVecState states,
                             shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                             double w_min = 0, QObject *parent = nullptr);

    virtual void step(const MatrixXd &z);


protected:
    shared_ptr<MatrixXi>    H;
    NestedPtrVecState       H_i;
    shared_ptr<VectorXd>    log_w;


#if RUN_TYPE == RUN_DEBUG
    friend class DebugServer;
#endif


};

#endif // MULTI_TRACKER_MHT_H
