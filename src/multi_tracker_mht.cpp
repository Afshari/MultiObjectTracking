#include "inc/multi_tracker_mht.h"

MultiTrackerMHT::MultiTrackerMHT(shared_ptr<Estimator> estimator, PtrVecState states,
                                   shared_ptr<Sensor> sensor, double gating_size, int reduction_M,
                                   double w_min, QObject *parent) :
    MultiTracker(estimator, states, sensor, gating_size, reduction_M, w_min, parent) {

    std::cout << "States Size: " << states->size() << std::endl;

    this->log_w = make_shared<VectorXd>(1);
    (*this->log_w)(0, 0) = 0;

    this->H = make_shared<MatrixXi>(1, states->size());
    for(int i = 0; i < states->size(); i++)
        (*this->H)(0, i) = 0;

    this->H_i = make_shared<vector<PtrVecState>>();
    for(int i = 0; i < states->size(); i++) {
        PtrVecState curr_state = make_shared<vector<shared_ptr<State>>>();
        curr_state->push_back( states->at(i) );
        this->H_i->push_back( curr_state );
    }
}


void MultiTrackerMHT::step(const MatrixXd &z, bool debug) {

    int n = this->H_i->size();
    int m = z.cols();

    //// 1. Ellipsoidal Gating
    vector<vector<ArrayXi>> gated_index;
    std::set<int> set_gated_index;
    for(int i = 0; i < n; i++) {
        int n_i = this->H_i->at(i)->size();
        // Utils::printf("n_i: %d", n_i);

        vector<ArrayXi> inner_gated_index;
        for(int lh = 0; lh < n_i; lh++) {
            auto gatingResult = this->estimator->ellipsoidalGating(*H_i->at(i)->at(lh), z, this->gating_size);
            MatrixXd gated_z  = *std::get<1>(gatingResult);
            ArrayXi curr_gated_index = *std::get<0>(gatingResult);

            inner_gated_index.push_back(curr_gated_index);
            for(int k = 0; k < curr_gated_index.size(); k++)
                set_gated_index.insert(curr_gated_index(k, 0));
        }
        gated_index.push_back(inner_gated_index);
    }
    std::cout << "End of Ellipsoidal Gating ..." << std::endl;

    vector<int> all_indices(m);
    std::generate(all_indices.begin(), all_indices.end(), [n = 0]() mutable { return n++; });
    set<int> set_all_indices(all_indices.begin(), all_indices.end());
    std::set<int> set_clutter_indices;
    std::set_difference(set_all_indices.begin(), set_all_indices.end(),
                        set_gated_index.begin(), set_gated_index.end(),
                        std::inserter(set_clutter_indices, set_clutter_indices.end()));

    m = set_gated_index.size();
    // Utils::printf("m: %d", m);
    MatrixXd new_z = z(Eigen::all, *Utils::setToArrayXi(set_gated_index));
    // Utils::printEigen<MatrixXd>(new_z, "new_z");

    if(debug == true) {
        int x = 0;
        x += 1;
        std::cout << "Dummy Print " << x << std::endl;
        std::cout << "Start Debugging ..." << std::endl;
    }

    NestedPtrVecState curr_H_i = make_shared<vector<PtrVecState>>(n);
    vector<VectorXd> curr_log_w_i(n);
    for(int i = 0; i < n; i++) {
        int n_i = this->H_i->at(i)->size();

        curr_log_w_i[i] = (-1) * MatrixXd::Constant(n_i*(m+1), 1, std::numeric_limits<double>::infinity());
        PtrVecState inner_H_i = make_shared<VecState>(n_i*(m+1));
        for(int lh = 0; lh < n_i; lh++) {

            int H_i_last_idx = 0;
            int new_z_last_idx = 0;
            for(int k = 0; k < gated_index[i][lh].rows(); k++) {
                int j = gated_index[i][lh](k, 0);
                int newidx = (lh)*(m+1) + j;
                int new_z_idx = j;
                if(newidx < n_i*(m+1)) {
                    H_i_last_idx = newidx;
                } else {
                    H_i_last_idx += 1;
                    newidx = H_i_last_idx;
                }
                if(j < m) {
                    new_z_last_idx = j;
                } else {
                    new_z_idx = new_z_last_idx;
                    new_z_last_idx += 1;
                }

                if(debug == true) {
                    Utils::printf("newidx: %d, lh: %d, m: %d, j: %d, n: %d", newidx, lh, m, j, n);
                }

                MatrixXd S = (*estimator->H(this->H_i->at(i)->at(lh)->getX())) *
                        this->H_i->at(i)->at(lh)->getP() *
                        estimator->H(this->H_i->at(i)->at(lh)->getX())->transpose();
                VectorXd zbar = *estimator->h(this->H_i->at(i)->at(lh)->getX());

                double formula_num_1 = log( sensor->get_P_D() / sensor->get_intensity() );
                double formula_num_2 = -0.5 * log( (2 * M_PI * S).determinant() );
                MatrixXd formula_mat_1 = -0.5 * (z(Eigen::all, j) - zbar).transpose() * S.inverse() * (z(Eigen::all, j) - zbar);
                double formula_num_3 = formula_mat_1(0, 0);

                curr_log_w_i[i](newidx, 0) = formula_num_1 + formula_num_2 + formula_num_3;
                if(debug == true) {
                    Utils::printEigen<VectorXd>(this->H_i->at(i)->at(lh)->getX(), "X");
                    Utils::printf("size of z: %d, %d", new_z.rows(), new_z.cols());
                    Utils::printEigen<MatrixXd>(new_z(Eigen::all, new_z_idx), "new z");
                    Utils::printf("size of inner_H_i: %d", inner_H_i->size());
                }
                inner_H_i->at(newidx) = estimator->update(*this->H_i->at(i)->at(lh), new_z(Eigen::all, new_z_idx));
            }
            int newidx = (lh+1)*(m+1)-1;
            curr_log_w_i[i](newidx, 0) = log( 1 - sensor->get_P_D() );
            inner_H_i->at(newidx) = this->H_i->at(i)->at(lh);
        }
        // Utils::printEigen<VectorXd>(curr_log_w_i[i], "curr_log_w_i[i]");
        curr_H_i->at(i) = inner_H_i;
//        for(uint k = 0; k < inner_H_i->size(); k++) {
//            if(inner_H_i->at(k) != nullptr)
//                Utils::printEigen<VectorXd>(inner_H_i->at(k)->getX(), "x");
//        }
    }
    std::cout << "End of Creating curr_H_i ... " << std::endl;

    vector<vector<int>> vec_H;
    vector<double> vec_w_logs;

    for(int h = 0; h < this->H->rows(); h++) {

        MatrixXd L = MatrixXd::Constant(n, n+m, std::numeric_limits<double>::infinity());
        for(int i = 0; i < n; i++) {

            int startingidx = (*this->H)(h, i) * (m+1);
            int finalidx    = ((*this->H)(h, i) + 1) * (m+1) - 2;
//            Utils::printf("m: %d       value: %d", m, (*this->H)(h, i));
//            Utils::printf("startingidx: %d       finalidx: %d", startingidx, finalidx);
            L.block(i, 0, 1, m) = -curr_log_w_i[i].block(startingidx, 0, finalidx-startingidx+1, 1).transpose();
            L(i, m+i) = -curr_log_w_i[i]( finalidx+1, 0 );
        }
        // std::cout << "End of Creating Cost Function ... index: " << h << std::endl;
        // Utils::printEigen<MatrixXd>(L, "L");

        //// 3. Run Murty Algorithm
        Murty murty(L);
        double cost;
        Assignment sol;
        vector<double> inner_vec_w_logs;
        vector<Assignment> assignment;
        int counter = 0;

        while(counter < this->reduction_M) {
            counter += 1;
            bool res = murty.draw(sol, cost);

            if(!res)  break;

            vec_w_logs.push_back(-cost + (*this->log_w)(h, 0) );
            inner_vec_w_logs.push_back(-cost + (*this->log_w)(h, 0) );
            // Utils::printEigen<MatrixXi>(sol, "sol");
            sol = (sol.array() > m).select(m, sol);
            assignment.push_back(sol);
        }
        assignment = *Hypothesis().sort(assignment, inner_vec_w_logs);
        int M = assignment.size();

        for(int iM = 0; iM < M; iM++) {
            vector<int> inner_H;
            for(int i = 0; i < n; i++) {
                int rs = (*this->H)(h, i) * (m+1) + assignment.at(iM)(i, 0);
                inner_H.push_back( rs );
            }
            vec_H.push_back( inner_H );
        }
    }

    shared_ptr<MatrixXi> curr_H = make_shared<MatrixXi>();
    curr_H->resize(vec_H.size(), n);

    for(uint i = 0; i < vec_H.size(); i++) {
        for(int j = 0; j < n; j++) {
            (*curr_H)(i, j) = vec_H[i][j];
        }
    }

    auto normalizeOutput = Hypothesis().normalizeLogWeights(vec_w_logs);
    VectorXd w_logs = *std::get<0>(normalizeOutput);
//    Utils::printEigen<VectorXd>(w_logs, "w_logs");

    vector<int> hyp(vec_w_logs.size());
    std::generate(hyp.begin(), hyp.end(), [n = 0]() mutable { return n++; });

    //// 4. Pruning Assignment Matrix
    auto pruneResult = Hypothesis().prune(hyp, w_logs, this->w_min);
    w_logs = *std::get<0>(pruneResult);
    hyp = *std::get<1>(pruneResult);
//    Utils::printEigen<VectorXd>(w_logs, "w_logs");
//    Utils::print<int>(hyp, "hyp");

    curr_H = Hypothesis::getWithIndices(curr_H, hyp);

    normalizeOutput = Hypothesis().normalizeLogWeights(w_logs);
    w_logs = *std::get<0>(normalizeOutput);

    hyp.resize( w_logs.rows() );
    std::generate(hyp.begin(), hyp.end(), [n = 0]() mutable { return n++; });
    auto capResult = Hypothesis().cap(hyp, w_logs, this->reduction_M);
    w_logs = *std::get<0>(capResult);
    hyp = *std::get<1>(capResult);

    curr_H = Hypothesis::getWithIndices(curr_H, hyp);
//    Utils::printEigen<MatrixXi>(*curr_H, "curr_H");

    normalizeOutput = Hypothesis().normalizeLogWeights(w_logs);
    w_logs = *std::get<0>(normalizeOutput);

    // Utils::printEigen<VectorXd>(w_logs, "w_logs");

    for(int i = 0; i < n; i++) {

        VectorXi curr_col_items = (*curr_H)(Eigen::all, i);
        set<int> set_curr_col_items(curr_col_items.data(), curr_col_items.data() + curr_col_items.rows());
        vector<int> vec_curr_col_items(set_curr_col_items.begin(), set_curr_col_items.end());
        // Utils::print<int>(set_curr_Col, "set_curr_Col");

        curr_H_i->at(i) = Utils::getWithIndices(*curr_H_i->at(i), vec_curr_col_items);
        curr_log_w_i.at(i) = *Utils::getWithIndices(curr_log_w_i.at(i), vec_curr_col_items);
        normalizeOutput = Hypothesis().normalizeLogWeights(curr_log_w_i.at(i));
        curr_log_w_i.at(i) = *std::get<0>(normalizeOutput);

        // Utils::printf("vec_curr_col_items count: %d", vec_curr_col_items.size());
        for(uint ri = 0; ri < vec_curr_col_items.size(); ri++) {
            VectorXi temporary_H = ( (*curr_H)(Eigen::all, i).array() == vec_curr_col_items[ri] ).select(ri, (*curr_H)(Eigen::all, i));
            (*curr_H)(Eigen::all, i) = temporary_H.transpose();
        }
    }
//    Utils::printEigen<MatrixXi>(*curr_H, "curr_H");

    MatrixXd::Index best_index;
    std::ignore = w_logs.maxCoeff(&best_index);
    std::cout << "best_index: " << best_index << std::endl;

    this->H = curr_H;
    this->H_i = curr_H_i;
    this->log_w = make_shared<VectorXd>(w_logs);

    for(int i = 0; i < n; i++) {
        Utils::printEigen<VectorXd>(curr_H_i->at(i)->at(best_index)->getX(), "state x");
    }

    NestedPtrVecState tt;

    for(int i = 0; i < n; i++) {
        int n_i = this->H_i->at(i)->size();
        for(int lh = 0; lh < n_i; lh++) {
            this->H_i->at(i)->at(lh) = estimator->predict(*this->H_i->at(i)->at(lh));
        }
    }

//    std::cout << "/////////////////" << std::endl;
//    for(int i = 0; i < n; i++) {
//        Utils::printEigen<VectorXd>(curr_H_i->at(i)->at(best_index)->getX(), "state x");
//    }
}















