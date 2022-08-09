#include "inc/multi_tracker_gnn.h"

MultiTrackerGNN::MultiTrackerGNN(shared_ptr<Estimator> estimator, PtrVecState states,
                       shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                       double w_min, QObject *parent) :
    MultiTracker(estimator, states, sensor, gating_size, reduction_M, w_min, parent) {

}

void MultiTrackerGNN::step(const MatrixXd &z, bool debug) {

    int n = this->states->size();
    //Utils::printEigen<VectorXd>(states->at(0)->getX(), "x");

    int m = z.cols();
    MatrixXi gated_index = MatrixXi::Zero(n, m);
    std::set<int> set_gated_index;
    for(int i = 0; i < n; i++) {
        auto gatingResult = this->estimator->ellipsoidalGating(*states->at(i), z, gating_size);
        MatrixXd gated_z = *std::get<1>(gatingResult);

        ArrayXi curr_gated_index = *std::get<0>(gatingResult);
        for(int j = 0; j < curr_gated_index.size(); j++)
            gated_index(i, curr_gated_index(j)) = 1;

        for(int k = 0; k < curr_gated_index.size(); k++)
            set_gated_index.insert(curr_gated_index(k, 0));
    }
    ArrayXi post_index(set_gated_index.size());
    int idx = 0;
    for(auto it = set_gated_index.begin(); it != set_gated_index.end(); ++it, ++idx) {
        post_index(idx) = *it;
    }
    MatrixXi post_gated_index = gated_index(Eigen::all, post_index);
    MatrixXd post_z = z(Eigen::all, post_index);

    m = post_gated_index.cols();

    MatrixXd L = MatrixXd::Constant(n, n+m, std::numeric_limits<double>::infinity());
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < post_gated_index.cols(); j++) {
            if(post_gated_index(i, j) == 1) {

                MatrixXd S_i_h    = (*estimator->H(states->at(i)->getX())) * states->at(i)->getP() * estimator->H(states->at(i)->getX())->transpose();
                VectorXd zbar_i_h = *estimator->h(states->at(i)->getX());

                double fn_1 = log( sensor->get_P_D() / sensor->get_intensity() );
                double fn_2 = -0.5 * log( (2 * M_PI * S_i_h).determinant() );
                MatrixXd fm_1 = -0.5 * (post_z(Eigen::all, j) - zbar_i_h).transpose() * S_i_h.inverse() * (post_z(Eigen::all, j) - zbar_i_h);
                double fn_3 = fm_1(0, 0);

                L(i, j) = -( fn_1 + fn_2 + fn_3  );
            }
        }
        L(i,m+i) = - log(1-sensor->get_P_D());
    }
    // Utils::printEigen<MatrixXd>(L, "L");

    Hungarian HungAlgo;
    vector<int> assignment;
    double cost;

    tie(assignment, cost) = HungAlgo.Solve(L);

    // Update
    for(uint i = 0; i < assignment.size(); i++) {
        if(assignment[i] < m) {
            states->at(i) = this->estimator->update(*states->at(i), post_z(Eigen::all, assignment[i]));
        }
    }

    if(print_result == true) {
        MatrixXd res(states->at(0)->getX().rows(), states->size());
        for(uint i = 0; i < states->size(); i++) {
         res(Eigen::all, i) = states->at(i)->getX();
        }
        std::cout << res.norm() << std::endl;
        //Utils::printEigen<MatrixXd>(res, "states");
        //Utils::printEigen<VectorXd>(states->at(0)->getX(), "x");
        //Utils::printEigen<VectorXd>(z(Eigen::all,assignment[0]), "z");
        // Utils::printEigen<MatrixXd>(states->at(0)->getP(), "P");
    }

    // Predict
    for(uint i = 0; i < states->size(); i++) {
        states->at(i) = this->estimator->predict(*states->at(i));
    }
}

VectorXd MultiTrackerGNN::getX(int idx) {

    return states->at(idx)->getX();
}









