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

    virtual void step(const MatrixXd &z, bool debug = false);
    virtual VectorXd getX(int idx);

    void ellipsoidalGating(vector<vector<ArrayXi>>& gated_index, std::set<int>& set_gated_index,
                           const MatrixXd &z, int n);
    void generate_H_w_i(NestedPtrVecState& curr_H_i, vector<VectorXd>& curr_log_w_i, MatrixXd& new_z,
                        vector<vector<ArrayXi>>& gated_index, std::set<int>& set_gated_index,
                        const MatrixXd &z, int m, int n);
    void generateCostMatrix(MatrixXd& L, const vector<VectorXd>& curr_log_w_i, int h, int n, int m);
    void runMurty(vector<vector<int>>& vec_H, vector<double>& vec_w_logs, const MatrixXd& L, int h, int n, int m);
    void prune(shared_ptr<MatrixXi>& curr_H, VectorXd& w_logs,
               tuple<shared_ptr<VectorXd>, double>& normalizeOutput, const vector<double>& vec_w_logs);
    void predict(int n);

protected:
    shared_ptr<MatrixXi>    H;
    NestedPtrVecState       H_i;
    shared_ptr<VectorXd>    log_w;
    MatrixXd::Index best_index;


#if RUN_TYPE == RUN_DEBUG
    friend class DebugServer;
#endif


};

#endif // MULTI_TRACKER_MHT_H
