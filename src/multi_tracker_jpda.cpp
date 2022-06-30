#include "inc/multi_tracker_jpda.h"

MultiTrackerJPDA::MultiTrackerJPDA(shared_ptr<Estimator> estimator, PtrVecState states,
                                   shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                                   double w_min, QObject *parent) :
    MultiTracker(estimator, states, sensor, gating_size, reduction_M, w_min, parent) {

}


void MultiTrackerJPDA::step(const MatrixXd &z, bool debug) {

    int n = this->states->size();

    //// 1. Run EllipsoidalGating on each State
    vector<ArrayXi> gated_index;
    std::set<int> set_gated_index;
    for(int i = 0; i < n; i++) {
        auto gatingResult = this->estimator->ellipsoidalGating(*states->at(i), z, gating_size);
        MatrixXd gated_z = *std::get<1>(gatingResult);

        ArrayXi curr_gated_index = *std::get<0>(gatingResult);
        // Utils::printEig<ArrayXi>(curr_gated_index, "curr_gated_index");

        gated_index.push_back(curr_gated_index);
        for(int k = 0; k < curr_gated_index.size(); k++)
            set_gated_index.insert(curr_gated_index(k, 0));
    }

    vector<int> vec_gated_index(set_gated_index.begin(), set_gated_index.end());
    sort(vec_gated_index.begin(), vec_gated_index.end());
    std::map<int, int> L_indices;
    for(int i = 0; i < set_gated_index.size(); i++) {
        L_indices[vec_gated_index[i]] = i;
    }
    //// 2. Create 'Cost Matrix' (L)
    int m = set_gated_index.size();
    MatrixXd L = MatrixXd::Constant(n, n+m, std::numeric_limits<double>::infinity());
    for(int i = 0; i < n; i++) {
        int gated_size = gated_index[i].size();
        for(int k = 0; k < gated_size; k++) {
            int j = gated_index.at(i)[k];
            int L_idx = L_indices[j];

            MatrixXd S    = (*estimator->H(states->at(i)->getX())) * states->at(i)->getP() * estimator->H(states->at(i)->getX())->transpose();
            VectorXd zbar = *estimator->h(states->at(i)->getX());

            double formula_num_1 = log( sensor->get_P_D() / sensor->get_intensity() );
            double formula_num_2 = -0.5 * log( (2 * M_PI * S).determinant() );
            MatrixXd formula_mat_1 = -0.5 * (z(Eigen::all, j) - zbar).transpose() * S.inverse() * (z(Eigen::all, j) - zbar);
            double formula_num_3 = formula_mat_1(0, 0);

            L(i, L_idx) = -( formula_num_1 + formula_num_2 + formula_num_3  );
        }
        L(i,m+i) = - log(1-sensor->get_P_D());
    }
    // Utils::printEigen<MatrixXd>(L, "L");

    //// 3. Run Murty Algorithm
    vector<double> vec_w_logs;
    Murty murty(L);
    double cost;
    Assignment sol;
    vector<Assignment> assignment;
    int counter = 0;

    while(counter < this->reduction_M) {

        counter += 1;
        bool res = murty.draw(sol, cost);

        if(!res)  break;

        vec_w_logs.push_back(-cost);
        sol = (sol.array() > m).select(m, sol);
        assignment.push_back(sol);
    }

    assignment = *Hypothesis().sort(assignment, vec_w_logs);

    auto normalizeOutput = Hypothesis().normalizeLogWeights(vec_w_logs);
    VectorXd w_logs = *std::get<0>(normalizeOutput);

    vector<int> hyp(vec_w_logs.size());
    std::generate(hyp.begin(), hyp.end(), [n = 0]() mutable { return n++; });

    //// 4. Pruning Assignment Matrix
    auto pruneResult = Hypothesis().prune(hyp, w_logs, this->w_min);
    w_logs = *std::get<0>(pruneResult);
    hyp = *std::get<1>(pruneResult);

    assignment = *Hypothesis::getWithIndices<Assignment>(assignment, hyp);

//    std::cout << "New Solutions" << std::endl;
//    for(auto sol : assignment) {
//        std::cout << sol << std::endl;
//        std::cout << "----------------------" << std::endl;
//    }

    normalizeOutput = Hypothesis().normalizeLogWeights(w_logs);
    w_logs = *std::get<0>(normalizeOutput);

    // Utils::printEigen<VectorXd>(w_logs, "w_logs");

    //// 5. Create Local Hypothesis
    MatrixXd beta = MatrixXd::Zero(n, m+1);
    for(int i = 0; i < n; i++) {
        for(uint theta_i = 0; theta_i < assignment.size(); theta_i++) {
            int j = (assignment[theta_i])(i, 0);
            beta(i,j) = beta(i,j) + exp( w_logs(theta_i, 0)  );
        }
    }
    // Utils::printEigen<MatrixXd>(beta, "beta");

    //// 6. Merge Local Hypothesis
    for(int i = 0; i < n; i++) {

        MatrixXd Hx = *estimator->H(states->at(i)->getX());
        MatrixXd S  = Hx * states->at(i)->getP() * Hx.transpose() + this->estimator->getR();
        MatrixXd K  = states->at(i)->getP() * Hx.transpose() * S.inverse();
        vector<VectorXd> ksi_ij;
        VectorXd ksi_i = VectorXd::Zero(z.rows(), 1);
        MatrixXd aux = MatrixXd::Zero(z.rows(), z.rows());
        for(int j = 0; j < m; j++) {
            VectorXd KS = z(Eigen::all, j) - ( *estimator->h( states->at(i)->getX() ) );
            ksi_ij.push_back(KS);
            ksi_i = ksi_i + beta(i,j) * KS;
            aux = aux + beta(i,j) * KS * KS.transpose();
        }
        VectorXd x = states->at(i)->getX() + K * ksi_i;
        MatrixXd P = beta(i, m)   * states->at(i)->getP() +
                (1 - beta(i, m))  * states->at(i)->getP() - K * S * K.transpose() +
                K * ( aux - ksi_i * ksi_i.transpose() ) * K.transpose();
        states->at(i) = make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );
    }

    if(print_result == true) {
        for(uint i = 0; i < states->size(); i++) {
            Utils::printEigen<VectorXd>(states->at(i)->getX(), "states");
        }
    }

    //// 7. Predict
    for(uint i = 0; i < states->size(); i++) {
        states->at(i) = this->estimator->predict(*states->at(i));
    }
}

VectorXd MultiTrackerJPDA::getX(int idx) {

    return states->at(idx)->getX();
}









