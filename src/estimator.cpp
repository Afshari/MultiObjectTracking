#include "inc/estimator.h"


Estimator::Estimator(QObject *parent) : QObject(parent)  {

    this->measurementModel = make_shared<MeasurementConstantVelocity>(0);
    this->transitionModel  = make_shared<TransitionConstantVelocity>(0, 0);
}

Estimator::Estimator(shared_ptr<MeasurementModel> measurementModel, shared_ptr<TransitionModel> transitionModel,
                 QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}


tuple<shared_ptr<ArrayXi>, shared_ptr<MatrixXd>> Estimator::ellipsoidalGating(
                                const State &state,
                                const MatrixXd &z, double gating_size) {

    // Hx = H( x )
    MatrixXd Hx = *this->measurementModel->H( state.getX() );
    // S = Hx @ P @ Hx.T + R
    MatrixXd S = Hx * state.getP() * Hx.transpose() + this->measurementModel->getR();
    // S = (S + S.T) / 2
    S = (S + S.transpose()) / 2;

    // zk = h( x )
    VectorXd zk = *this->measurementModel->h( state.getX() );

    // d = (zk - z)
    MatrixXd d = z - zk.replicate(1, z.cols());

    // dm = d.T @ inv(S) @ d
    VectorXd dm(z.cols());
    for(auto i = 0U; i < z.cols(); i++) {
        dm(i) = d(Eigen::all, i).transpose() * S.inverse() * d(Eigen::all, i);
    }

    MatrixXd z_gate = (dm.array() < gating_size).cast<double>().matrix();

    int len = (z_gate.array() > 0).colwise().count()[0];
    ArrayXi idx_in_gate(len);
    int idd_index = 0;
    for(auto i = 0U; i < z_gate.rows(); i++) {
        if(z_gate(i, 0) > 0) {
            idx_in_gate[idd_index] = i;
            idd_index++;
        }
    }

    shared_ptr<ArrayXi>  result_idx = make_shared<ArrayXi>( idx_in_gate );
    shared_ptr<MatrixXd> result_z = make_shared<MatrixXd>( z(Eigen::all, idx_in_gate) );

    tuple<shared_ptr<ArrayXi>, shared_ptr<MatrixXd>> result(result_idx, result_z);

    return result;
}


shared_ptr<VectorXd> Estimator::predictedLikelihood( const State &state,
                                                   const MatrixXd &z) {

    // Hx = H( x )
    MatrixXd Hx = *this->measurementModel->H( state.getX() );
    // S = Hx @ P @ Hx.T + R
    MatrixXd S = Hx * state.getP() * Hx.transpose() + this->measurementModel->getR();
    // S = (S + S.T) / 2
    S = (S + S.transpose()) / 2;

    // zk = h( x )
    VectorXd zk = *this->measurementModel->h( state.getX() );
    // VectorXd zk = Hx * state.getX();

    VectorXd likelihood(z.cols());

    for(auto i = 0U; i < z.cols(); i++) {
        likelihood(i, 0) = stats::dmvnorm<VectorXd, MatrixXd>(z(Eigen::all, i), zk, S, true);
    }

    return make_shared<VectorXd>( likelihood );
}


shared_ptr<State> Estimator::momentMatching(const VectorXd &w, const VecState &states) {


    int mk = w.rows();
    if(mk == 1) {
        return make_shared<State>( *(states[0]) );
    }

    VectorXd new_w = w.array().exp();

    VectorXd x = VectorXd::Zero( states[0]->getX().rows() );
    for(int i = 0; i < mk; i++) {
        // x = x + w * state.x
        x = x + new_w[i] * states[i]->getX();
    }

    MatrixXd P = MatrixXd::Zero(states[0]->getP().rows(), states[0]->getP().cols());
    for(int i = 0; i < mk; i++) {
        // P = P + w * state.P + w * (x - state.x) @ (x - state.x).T
        P = P + new_w[i] * states[i]->getP() + new_w[i] * (x - states[i]->getX()) * (x - states[i]->getX()).transpose();
    }

    return make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );
}

State Estimator::momentMatching(const VectorXd &w, const shared_ptr<vector<State>>& states) {


    int mk = w.rows();
    if(mk == 1) {
        return (states->at(0));
    }

    VectorXd new_w = w.array().exp();

    VectorXd x = VectorXd::Zero( states->at(0).getX().rows() );
    for(int i = 0; i < mk; i++) {
        // x = x + w * state.x
        x = x + new_w[i] * states->at(i).getX();
    }

    MatrixXd P = MatrixXd::Zero(states->at(0).getP().rows(), states->at(0).getP().cols());
    for(int i = 0; i < mk; i++) {
        // P = P + w * state.P + w * (x - state.x) @ (x - state.x).T
        P = P + new_w[i] * states->at(i).getP() + new_w[i] * (x - states->at(i).getX()) * (x - states->at(i).getX()).transpose();
    }

    return State( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );
}

shared_ptr<State> Estimator::predict(const State &state) {

    // x = f( x )
    shared_ptr<VectorXd> x = transitionModel->f( state.getX() );
    // P = F(x) @ P @ F(x).T + Q
    MatrixXd P = ( *transitionModel->F( state.getX() ) ) * state.getP() * transitionModel->F( state.getX() )->transpose() +
            (*transitionModel->Q());

    return make_shared<State>(x, make_shared<MatrixXd>(P));
}


shared_ptr<State> Estimator::update(const State &state, const MatrixXd &z) {

    // Hx = H( x )
    MatrixXd Hx = *measurementModel->H(state.getX());
    // S = Hx @  P * Hx.T + R
    MatrixXd S = Hx * state.getP() * Hx.transpose() + measurementModel->getR();
    S = (S + S.transpose()) / 2;
    // K = P @ Hx.T @ inv(S)
    MatrixXd K = state.getP() * Hx.transpose() * S.inverse();

    // zk = h( x )
    VectorXd zk = *this->measurementModel->h( state.getX() );
    VectorXd x = state.getX() + K * (z - zk);
    MatrixXd P = ( MatrixXd::Identity(x.rows(), x.rows()) - K * Hx ) * state.getP();

    return make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );
}

shared_ptr<VectorXd> Estimator::h(const VectorXd &x) {

    return this->measurementModel->h(x);
}


shared_ptr<MatrixXd> Estimator::H(const VectorXd &x) {

    return this->measurementModel->H(x);
}


MatrixXd Estimator::getR() {

    return  this->measurementModel->getR();
}










