#ifndef TRANSITION_MODEL_H
#define TRANSITION_MODEL_H

#include <QObject>
#include <QDebug>
#include <memory>
#include <Eigen/Dense>

using std::shared_ptr;
using std::make_shared;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class TransitionModel : public QObject {
    Q_OBJECT
public:
    explicit TransitionModel(float T, QObject *parent = nullptr);
    virtual MatrixXd transitionFunction(int dt) { std::ignore = dt; return MatrixXd(); }
    virtual MatrixXd transitionCov(int dt) { std::ignore = dt; return MatrixXd(); }

    virtual shared_ptr<MatrixXd> F(const VectorXd &x)
            { std::ignore = x; return make_shared<MatrixXd>( MatrixXd() ); }
    virtual shared_ptr<VectorXd> f(const VectorXd &x)
            { std::ignore = x; return make_shared<VectorXd>( VectorXd() ); }
    virtual shared_ptr<MatrixXd> Q()  { return make_shared<MatrixXd>( MatrixXd() ); }

protected:
    float T;
    shared_ptr<MatrixXd> _Q;


};

#endif // TRANSITION_MODEL_H
