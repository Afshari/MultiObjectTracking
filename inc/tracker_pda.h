#ifndef TRACKER_PDA_H
#define TRACKER_PDA_H

#include <QObject>

#include "inc/tracker.h"
#include "inc/hypothesis.h"
#include "inc/utils.h"

using std::shared_ptr;
using std::make_shared;
using Eigen::Vector;

class TrackerPDA : public Tracker {
    Q_OBJECT
public:
    explicit TrackerPDA(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                        shared_ptr<Sensor> sensor, double gating_size, double w_min = 0,
                        QObject *parent = nullptr);
    virtual void step(const MatrixXd &z);

protected:
    shared_ptr<State> state;

};

#endif // TRACKER_PDA_H
