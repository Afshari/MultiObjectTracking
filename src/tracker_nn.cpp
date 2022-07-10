#include "inc/tracker_nn.h"

TrackerNN::TrackerNN(shared_ptr<Estimator> estimator, shared_ptr<State> initial_state,
                     shared_ptr<Sensor> sensor,   double gating_size, double w_min, QObject *parent) :
    Tracker(estimator, initial_state, sensor, gating_size, 0, w_min, parent) {

}

void TrackerNN::step(const MatrixXd &z) {

    shared_ptr<State> upd_state;
    shared_ptr<State> pred_state = this->state;

    // VecState states;
    // states.push_back(this->state);
    shared_ptr<ArrayXi> z_gate_idx;
    shared_ptr<MatrixXd> z_gate;
    tie(z_gate_idx, z_gate) = estimator->ellipsoidalGating(*this->state, z, gating_size);

    int mk = z_gate_idx->rows() + 1;
    // Utils::printEigen<MatrixXd>(*z_gate, "z gate");

    VectorXd predicted_likelihood = (*estimator->predictedLikelihood(*this->state, *z_gate)).array().exp();
    // Utils::printEigen<VectorXd>(predicted_likelihood, "Predicted Likelihood");

    VectorXd w_theta_k = this->sensor->get_P_D()  * predicted_likelihood / this->sensor->get_intensity();
    // Utils::printEigen<VectorXd>(w_theta_k, "w_theta_k");

    double w_theta_0 = 1 - this->sensor->get_P_D();

    if(mk == 1) {
        upd_state = pred_state;
    } else {

        MatrixXd::Index max_index;
        double max_theta = w_theta_k.maxCoeff(&max_index);
        // Utils::printf("max: %d   %f\r\n", max_index, max_theta);

        if(w_theta_0 > max_theta) {
            upd_state = pred_state;
        } else {

            MatrixXd z_NN = (*z_gate)(Eigen::all, max_index);
            // Utils::printEigen<MatrixXd>(z_NN, "z_NN");
            upd_state = this->estimator->update(*this->state, z_NN);
        }
    }
    this->updated_x = upd_state->getX();
    // Utils::printEigen<VectorXd>(upd_state->getX(), "update state");
    // std::cout << std::endl;

    pred_state = this->estimator->predict(*upd_state);
    this->state = pred_state;
    // Utils::printEigen<VectorXd>(pred_state->getX(), "predict state");
    // std::cout << std::endl;
}







