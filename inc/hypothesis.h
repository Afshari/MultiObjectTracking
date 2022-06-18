#ifndef HYPOTHESIS_H
#define HYPOTHESIS_H

#include <QObject>
#include <Eigen/Dense>
#include <cmath>

#include "inc/state.h"
#include "inc/utils.h"
#include "inc/estimator.h"
#include "libs/murty/murty.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXi;
using std::shared_ptr;
using std::make_shared;
using std::vector;
using std::tuple;

class Hypothesis : public QObject {
    Q_OBJECT
public:
    explicit Hypothesis(QObject *parent = nullptr);

    virtual tuple<shared_ptr<VectorXd>, PtrVecState> prune(const VecState &states, const VectorXd &weights, float threshold);
    virtual tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> prune(const vector<int> &states, const VectorXd &weights, float threshold);
    virtual tuple<shared_ptr<VectorXd>, PtrVecState> cap(const VecState &states, const VectorXd &weights, int threshold);
    virtual tuple<shared_ptr<VectorXd>, shared_ptr<vector<int>>> cap(const vector<int> &states, const VectorXd &weights, int threshold);
    virtual tuple<shared_ptr<VectorXd>, PtrVecState> merge(const VecState &states, const VectorXd &weights, long long threshold);
    virtual shared_ptr<vector<Assignment>> sort(const vector<Assignment> &assignment, vector<double> &costs);

    template<typename T>
    static shared_ptr<vector<T>> getWithIndices(const vector<T> &vec, const vector<int> &indices);
    static shared_ptr<MatrixXi> getWithIndices(const shared_ptr<MatrixXi> &mat, vector<int> &indices);

    virtual tuple<shared_ptr<VectorXd>, double> normalizeLogWeights(const VectorXd &log_w);
    virtual tuple<shared_ptr<VectorXd>, double> normalizeLogWeights(const vector<double> &log_w);


};

#endif // HYPOTHESIS_H
