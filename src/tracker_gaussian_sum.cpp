#include "inc/tracker_gaussian_sum.h"

TrackerGaussianSum::TrackerGaussianSum(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                                       shared_ptr<Sensor> sensor, double gating_size, double reduction_M,
                                       double w_min, QObject *parent) :
    Tracker(estimator, initial_state, sensor, gating_size, reduction_M, w_min, parent) {

}


void TrackerGaussianSum::step(const MatrixXd &z) {

    double w_theta_log_0 = log( 1 - this->sensor->get_P_D() );
    Vector<double, 1> w_theta_factor( log( this->sensor->get_P_D() / this->sensor->get_intensity() ) );

    // Utils::print(*this->w_logs, "w logs");
    // Utils::printf("Hypothesis Size: %d", this->hypotheses->size());

    VecState hypotheses_update;
    VectorXd w_log_update;

    for(auto theta = 0U; theta < this->hypotheses->size(); theta++) {

        shared_ptr<State> state = this->hypotheses->at(theta);
        double w_log = (*w_logs)(theta);

        auto gatingResult = this->estimator->ellipsoidalGating(*state, z, gating_size);
        MatrixXd gated_z = *std::get<1>(gatingResult);
        int mk = gated_z.cols();
        // Utils::print(z_gate, "z_gate");
        shared_ptr<State> hypothesis_0 = state;

        shared_ptr<VectorXd> log_likelihood = this->estimator->predictedLikelihood(*state, gated_z);
        // Utils::print(*log_likelihood, "log_likelihood");
        VectorXd w_theta_log_k;
        if(log_likelihood->rows() > 0) {
            w_theta_log_k = Vector<double, 1>(w_log).replicate(log_likelihood->rows(), 1) +
                    (*log_likelihood) + w_theta_factor.replicate(log_likelihood->rows(), 1);
        }

        w_theta_log_k.conservativeResizeLike(VectorXd::Zero(mk+1));
        w_theta_log_k(mk, 0) = w_log + w_theta_log_0;

        VecState hypotheses_mk;
        for(auto i = 0; i < mk; i++) {
            hypotheses_mk.push_back( this->estimator->update(*state, gated_z(Eigen::all, i) ) );
        }
        hypotheses_mk.push_back(hypothesis_0);

        std::copy(hypotheses_mk.begin(), hypotheses_mk.end(), std::back_inserter(hypotheses_update));
        int w_log_update_curr_size = w_log_update.rows();
        w_log_update.conservativeResizeLike(VectorXd::Zero(w_log_update.rows()+w_theta_log_k.rows()));
        w_log_update << w_log_update.block(0, 0, w_log_update_curr_size, 1), w_theta_log_k;
    }

    auto normOutput = Hypothesis().normalizeLogWeights(w_log_update);
    w_log_update = *std::get<0>(normOutput);

    auto pruneResult = Hypothesis().prune(hypotheses_update, w_log_update, this->w_min);
    w_log_update = *std::get<0>(pruneResult);
    hypotheses_update = *std::get<1>(pruneResult);

    normOutput = Hypothesis().normalizeLogWeights(w_log_update);
    w_log_update = *std::get<0>(normOutput);


    auto mergeResult = Hypothesis().merge(hypotheses_update, w_log_update, 4);
    w_log_update = *std::get<0>(mergeResult);
    hypotheses_update = *std::get<1>(mergeResult);

    auto capResult = Hypothesis().cap(hypotheses_update, w_log_update, this->reduction_M);
    w_log_update = *std::get<0>(capResult);
    hypotheses_update = *std::get<1>(capResult);

    normOutput = Hypothesis().normalizeLogWeights(w_log_update);
    w_log_update = *std::get<0>(normOutput);

    // Utils::print(w_log_update, "w_log_update");

    MatrixXd::Index max_index;
    double max_theta = w_log_update.maxCoeff(&max_index);
    Utils::printf("max: %d   %f\r\n", max_index, max_theta);
    Utils::printEigen<VectorXd>(hypotheses_update[max_index]->getX(), "state x");

    for(auto theta = 0U; theta < hypotheses_update.size(); theta++) {
        hypotheses_update[theta] = this->estimator->predict(*hypotheses_update[theta]);
    }

    // for(auto i = 0U; i < hypotheses_update.size(); i++)
        // Utils::print(hypotheses_update[i]->getX(), "predicted x");

}














