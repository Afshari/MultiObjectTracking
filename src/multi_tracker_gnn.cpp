#include "inc/multi_tracker_gnn.h"

MultiTrackerGNN::MultiTrackerGNN(shared_ptr<Estimator> estimator, PtrVecState states,
                       shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                       double w_min, QObject *parent) :
    MultiTracker(estimator, states, sensor, gating_size, reduction_M, w_min, parent) {

}

void MultiTrackerGNN::step(const MatrixXd &z) {

    int n = this->states->size();
    Utils::printf("gating_size: %f", gating_size);

    vector<ArrayXi> gated_index;
    std::set<int> set_gated_index;
    for(int i = 0; i < n; i++) {
        auto gatingResult = this->estimator->ellipsoidalGating(*states->at(i), z, gating_size);
        MatrixXd gated_z = *std::get<1>(gatingResult);

        ArrayXi curr_gated_index = *std::get<0>(gatingResult);
//        int curr_gated_size = gated_index.size();

        gated_index.push_back(curr_gated_index);

        for(int k = 0; k < curr_gated_index.size(); k++)
            set_gated_index.insert(curr_gated_index(k, 0));
    }

//    std::cout << "set_gated_index ";
//    for(auto idx : set_gated_index)
//        std::cout << idx << " ";
//    std::cout << std::endl;

    int m = set_gated_index.size();

//    std::cout << "gated_index" << std::endl;
    MatrixXd L = MatrixXd::Constant(n, n+m, std::numeric_limits<double>::infinity());
    for(int i = 0; i < n; i++) {
        int gated_size = gated_index[i].size();
        for(int k = 0; k < gated_size; k++) {
            int j = gated_index.at(i)[k];

            MatrixXd S_i_h    = (*estimator->H(states->at(i)->getX())) * states->at(i)->getP() * estimator->H(states->at(i)->getX())->transpose();
            VectorXd zbar_i_h = *estimator->h(states->at(i)->getX());

            double formula_num_1 = log( sensor->get_P_D() / sensor->get_intensity() );
            double formula_num_2 = -0.5 * log( (2 * M_PI * S_i_h).determinant() );
            MatrixXd formula_mat_1 = -0.5 * (z(Eigen::all, j) - zbar_i_h).transpose() * S_i_h.inverse() * (z(Eigen::all, j) - zbar_i_h);
            double formula_num_3 = formula_mat_1(0, 0);

            L(i, j) = -( formula_num_1 + formula_num_2 + formula_num_3  );
        }
        L(i,m+i) = - log(1-sensor->get_P_D());
    }
    // Utils::printEig<MatrixXd>(L, "L");

    Hungarian HungAlgo;
    vector<int> assignment;
    double cost;

    tie(assignment, cost) = HungAlgo.Solve(L);

    // Update
    for(uint i = 0; i < assignment.size(); i++) {
        if(assignment[i] < m) {
            states->at(i) = this->estimator->update(*states->at(i), z(Eigen::all,assignment[i]));
        }
    }

    // Predict
    for(uint i = 0; i < states->size(); i++) {
        states->at(i) = this->estimator->predict(*states->at(i));
    }

    for(uint i = 0; i < states->size(); i++) {
        Utils::printEigen<VectorXd>(states->at(i)->getX(), "states");
    }

}











