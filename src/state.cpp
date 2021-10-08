#include "inc/state.h"

State::State(shared_ptr<VectorXd> x, shared_ptr<MatrixXd> P, QObject *parent) : QObject(parent) {

    this->x = x;
    this->P = P;
//    std::cout << "Default Constructor ..." << std::endl;
}

State::State(const State &state) {

    this->x = make_shared<VectorXd>( state.getX() );
    this->P = make_shared<MatrixXd>( state.getP() );
    std::cout << "Copy Constructor ..." << std::endl;
}


VectorXd State::getX() const {
    return *this->x;
}

MatrixXd State::getP() const {
    return *this->P;
}
