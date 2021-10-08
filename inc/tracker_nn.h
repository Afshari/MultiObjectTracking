#ifndef TRACKER_NN_H
#define TRACKER_NN_H

#include <QObject>
#include <Eigen/Dense>

#include "inc/tracker.h"
#include "inc/state.h"
#include "inc/estimator.h"
#include "inc/utils.h"

using std::shared_ptr;
using std::make_shared;
using Eigen::Vector2d;

class TrackerNN : public Tracker {
    Q_OBJECT
public:
    explicit TrackerNN(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                       shared_ptr<Sensor> sensor, double gating_size, double w_min = 0,
                       QObject *parent = nullptr);
    virtual void step(const MatrixXd &z);



};

#endif // TRACKER_NN_H
