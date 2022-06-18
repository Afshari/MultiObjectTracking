#include "inc/hypothesis.h"

Hypothesis::Hypothesis(QObject *parent) : QObject(parent) {

}


tuple<shared_ptr<VectorXd>, PtrVecState> Hypothesis::prune(const VecState &states,
                                                           const VectorXd &weights, float threshold) {

    int len = (weights.array() > threshold).colwise().count()[0];

    ArrayXi idx_arr(len);
    int idd_index = 0;
    for(auto i = 0U; i < weights.rows(); i++) {
        if(weights(i, 0) > threshold) {
            idx_arr[idd_index] = i;
            idd_index++;
        }
    }

    VectorXd n_weights = weights(idx_arr, 0);
    VecState n_states;

    vector<int> idx_vec(idx_arr.data(), idx_arr.data() + idx_arr.rows());
    for(auto i = 0U; i < idx_vec.size(); i++) {
        int idx = idx_vec[i];
        n_states.push_back( states[idx] );
    }

    tuple<shared_ptr<VectorXd>, PtrVecState> result( make_shared<VectorXd>(n_weights),
                                                     make_shared<VecState>(n_states) );

    return result;
}

tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> Hypothesis::prune(const vector<int> &states,
                                                           const VectorXd &weights, float threshold) {

    int len = (weights.array() > threshold).colwise().count()[0];

    ArrayXi idx_arr(len);
    int idd_index = 0;
    for(auto i = 0U; i < weights.rows(); i++) {
        if(weights(i, 0) > threshold) {
            idx_arr[idd_index] = i;
            idd_index++;
        }
    }

    VectorXd n_weights = weights(idx_arr, 0);
    vector<int> n_states;

    vector<int> idx_vec(idx_arr.data(), idx_arr.data() + idx_arr.rows());
    for(auto i = 0U; i < idx_vec.size(); i++) {
        int idx = idx_vec[i];
        n_states.push_back( states[idx] );
    }

    tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> result(
                                    make_shared<VectorXd>(n_weights),
                                    make_shared<vector<int>>(n_states) );

    return result;
}



tuple<shared_ptr<VectorXd>, PtrVecState> Hypothesis::cap(const VecState &states, const VectorXd &weights, int threshold) {

    vector<double> vec_weights(weights.data(), weights.data() + weights.rows());

    vector<int> indcies(vec_weights.size());
    std::iota( indcies.begin(), indcies.end(), 0 );
    std::sort( indcies.begin(), indcies.end(), [&](int i,int j) { return vec_weights[i] > vec_weights[j]; } );

    std::sort(vec_weights.begin(), vec_weights.end(), std::greater<>());
    threshold = std::min<int>(threshold, vec_weights.size());
    vector<double> result_weights(vec_weights.begin(), vec_weights.begin() + threshold);
    vector<int> result_indices(indcies.begin(), indcies.begin() + threshold);

    VecState result_states;
    for(auto i = 0U; i < result_indices.size(); i++) {
        result_states.push_back( states[result_indices[i]] );
    }

    VectorXd n_weights = Eigen::Map<VectorXd, Eigen::Unaligned>(result_weights.data(), result_weights.size());

    tuple<shared_ptr<VectorXd>, PtrVecState> result( make_shared<VectorXd>(n_weights),
                                                     make_shared<VecState>(result_states) );

    return result;
}

tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> Hypothesis::cap(const vector<int> &states, const VectorXd &weights, int threshold) {

    vector<double> vec_weights(weights.data(), weights.data() + weights.rows());

    vector<int> indcies(vec_weights.size());
    std::iota( indcies.begin(), indcies.end(), 0 );
    std::sort( indcies.begin(), indcies.end(), [&](int i,int j) { return vec_weights[i] > vec_weights[j]; } );

    std::sort(vec_weights.begin(), vec_weights.end(), std::greater<>());
    threshold = std::min<int>(threshold, vec_weights.size());
    vector<double> result_weights(vec_weights.begin(), vec_weights.begin() + threshold);
    vector<int> result_indices(indcies.begin(), indcies.begin() + threshold);

    vector<int> result_states;
    for(auto i = 0U; i < result_indices.size(); i++) {
        result_states.push_back( states[result_indices[i]] );
    }

    VectorXd n_weights = Eigen::Map<VectorXd, Eigen::Unaligned>(result_weights.data(), result_weights.size());

    tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> result( make_shared<VectorXd>(n_weights),
                                                                 make_shared<vector<int>>(result_states) );

    return result;
}


tuple<shared_ptr<VectorXd>, PtrVecState> Hypothesis::merge(const VecState &states, const VectorXd &weights, long long threshold) {

    vector<int> I(weights.rows());
    std::iota(I.begin(), I.end(), 0);

    vector<double> w(weights.data(), weights.data() + weights.rows());
    PtrVecState states_hat = make_shared<VecState>();
    vector<double> w_hat;

    while(I.size() > 0) {

        int j = std::max_element(w.begin(), w.end()) - w.begin();
        vector<int> Ij;

        for(auto i : I) {
            VectorXd temp = states[i]->getX() - states[j]->getX();
            MatrixXd val  = states[j]->getP().fullPivHouseholderQr().solve(temp);
            MatrixXd tt   = (temp.transpose() * val).diagonal();

            if(tt(0, 0) < threshold)
                Ij.push_back(i);
        }

        // Utils::print(Ij, "Ij");
        shared_ptr<VectorXd> temp;
        double sum_log;
        tie(temp, sum_log) = normalizeLogWeights(*Utils::getWithIndices(w, Ij));
        w_hat.push_back(sum_log);
        shared_ptr<State> state = Estimator().momentMatching(*temp, *Utils::getWithIndices(states, Ij));
        states_hat->push_back(state);

        I.erase(std::remove_if(I.begin(), I.end(), [&Ij](const auto &x) {
            return std::find(Ij.begin(), Ij.end(), x) != Ij.end();
        }), I.end());

        // w(weights.data(), weights.data() + weights.rows())
        double log_epsilon = log(std::numeric_limits<double>::epsilon());
        for(auto i : Ij)
            w[i] = log_epsilon;
    }

    VectorXd v_w_hat = Eigen::Map<VectorXd, Eigen::Unaligned>(w_hat.data(), w_hat.size());
    tuple<shared_ptr<VectorXd>, PtrVecState> result(make_shared<VectorXd>(v_w_hat), states_hat);

    return result;
}


tuple<shared_ptr<VectorXd>, double> Hypothesis::normalizeLogWeights(const VectorXd &log_w) {

    VectorXd result_log_w = VectorXd::Zero(log_w.rows());
    double sum_log_w = 0;

    if(log_w.rows() <= 1) {

        sum_log_w = log_w(0, 0);
        result_log_w(0, 0) = log_w(0, 0) - sum_log_w;
    } else {

        vector<double> vec_log_w(log_w.data(), log_w.data() + log_w.rows());

        std::sort(vec_log_w.begin(), vec_log_w.end(), std::greater<>());

        VectorXd v_t1 = Eigen::Map<VectorXd, Eigen::Unaligned>(vec_log_w.data(), vec_log_w.size());
        VectorXd v_t2 = v_t1(Eigen::seq(1, Eigen::last)) - Vector<double, 1>( v_t1.maxCoeff() ).replicate(v_t1.rows()-1, 1);
        sum_log_w = v_t1.maxCoeff() + log( 1 + v_t2.array().exp().sum() );
        result_log_w = log_w - Vector<double, 1>(sum_log_w).replicate(log_w.rows(), 1);
    }

    tuple<shared_ptr<VectorXd>, double> result(make_shared<VectorXd>(result_log_w), sum_log_w);
    return  result;
}

tuple<shared_ptr<VectorXd>, double> Hypothesis::normalizeLogWeights(const vector<double> &log_w) {

    vector<double> n_log_w;
    n_log_w.assign(log_w.begin(), log_w.end());
    VectorXd v_log_w = Eigen::Map<VectorXd, Eigen::Unaligned>(n_log_w.data(), n_log_w.size());

    return  normalizeLogWeights(v_log_w);
}


shared_ptr<vector<Assignment>> Hypothesis::sort(const vector<Assignment> &assignment, vector<double> &costs) {

    // Utils::print<double>(costs, "costs");
    vector<int> indcies(costs.size());
    std::iota( indcies.begin(), indcies.end(), 0 );
    std::sort( indcies.begin(), indcies.end(), [&](int i,int j) { return costs[i] > costs[j]; } );
    std::sort(costs.begin(), costs.end(), std::greater<>());
    // Utils::print<double>(costs, "costs");

    vector<Assignment> solution;
    for(uint i = 0; i < assignment.size(); i++) {
        solution.push_back(assignment[indcies[i]]);
    }

    return make_shared<vector<Assignment>>(solution);
}


template<typename T>
shared_ptr<vector<T>> Hypothesis::getWithIndices(const vector<T> &vec, const vector<int> &indices) {

    shared_ptr<vector<T>> result = make_shared<vector<T>>();
    result->reserve(indices.size());

    for (auto index : indices)
        result->push_back( ( vec[index] ) );

    return result;
}

shared_ptr<MatrixXi> Hypothesis::getWithIndices(const shared_ptr<MatrixXi> &mat, vector<int> &indices) {

    ArrayXi arr_indices = Eigen::Map<Eigen::ArrayXi, Eigen::Unaligned>(indices.data(), indices.size());
    return make_shared<MatrixXi>( (*mat)(arr_indices, Eigen::all) );
}

template shared_ptr<vector<double>> Hypothesis::getWithIndices(const vector<double> &vec, const vector<int> &indices);
template shared_ptr<vector<Assignment>> Hypothesis::getWithIndices(const vector<Assignment> &vec, const vector<int> &indices);



















