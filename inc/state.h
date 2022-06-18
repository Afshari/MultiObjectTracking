#ifndef STATE_H
#define STATE_H

#include <QObject>
#include <Eigen/Dense>
#include <iostream>
#include <memory>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::shared_ptr;
using std::make_shared;
using std::vector;

class State : public QObject {
    Q_OBJECT
public:
    explicit State(shared_ptr<VectorXd> x, shared_ptr<MatrixXd> P, QObject *parent = nullptr);

    State(const State &state);

    virtual VectorXd getX() const;
    virtual MatrixXd getP() const;

protected:
    shared_ptr<VectorXd> x;
    shared_ptr<MatrixXd> P;

};

using VecState    = vector<shared_ptr<State>>;
using PtrVecState = shared_ptr<vector<shared_ptr<State>>>;
using NestedPtrVecState = shared_ptr<vector<PtrVecState>>;


#endif // STATE_H
