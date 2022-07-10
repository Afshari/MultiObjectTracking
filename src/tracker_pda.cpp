#include "inc/tracker_pda.h"

TrackerPDA::TrackerPDA(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                       shared_ptr<Sensor> sensor, double gating_size, double w_min, QObject *parent) :
    Tracker(estimator, initial_state, sensor, gating_size, 0, w_min, parent) {

}


void TrackerPDA::step(const MatrixXd &z) {

    double w_theta_log_0 = log( 1 - this->sensor->get_P_D() );
    Vector<double, 1> w_theta_factor( log( this->sensor->get_P_D() / this->sensor->get_intensity() ) );

    shared_ptr<ArrayXi> z_gate_idx;
    shared_ptr<MatrixXd> z_gate;
    tie(z_gate_idx, z_gate) = estimator->ellipsoidalGating(*this->state, z, gating_size);
    int mk = z_gate->cols();

    shared_ptr<VectorXd> log_likelihood = estimator->predictedLikelihood(*this->state, *z_gate);

    VectorXd w_theta_log_k = (*log_likelihood) + w_theta_factor.replicate((*log_likelihood).rows(), 1);
    w_theta_log_k.conservativeResizeLike(MatrixXd::Zero(mk+1, 1));
    w_theta_log_k(mk, 0) = w_theta_log_0;
    // Utils::print(w_theta_log_k, "w_theta_log_k");

    w_theta_log_k = *std::get<0>(Hypothesis().normalizeLogWeights(w_theta_log_k));
    // Utils::print(w_theta_log_k, "w_theta_log_k");

    VecState states;
    for(auto i = 0U; i < z_gate->cols(); i++) {
        states.push_back( this->estimator->update(*this->state, (*z_gate)(Eigen::all, i)) );
    }
    states.push_back( this->state );

    auto pruneResult = Hypothesis().prune(states, w_theta_log_k, this->w_min);
    w_theta_log_k = *std::get<0>(pruneResult);
    VecState hypotheses = *std::get<1>(pruneResult);

     w_theta_log_k = *std::get<0>(Hypothesis().normalizeLogWeights(w_theta_log_k));

     auto mergeResult = Hypothesis().merge(hypotheses, w_theta_log_k, 10000000000000);
     w_theta_log_k = *std::get<0>(mergeResult);
     hypotheses = *std::get<1>(mergeResult);

     // Utils::printEigen<VectorXd>(w_theta_log_k, "w_theta_log_k");

     // Utils::printf("h size %d", hypotheses.size());
     this->state = hypotheses[0];
     this->updated_x = this->state->getX();
     // Utils::printEigen<VectorXd>(this->state->getX(), "updated state");

     this->state = this->estimator->predict(*this->state);
     // Utils::printEigen<VectorXd>(this->state->getX(), "this state");
}












