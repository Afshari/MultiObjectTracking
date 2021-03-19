#include "state.h"

State::State(QObject *parent) : QObject(parent) {

}

Eigen::VectorXd State::getX() {
    return *this->x;
}

Eigen::MatrixXd State::getP() {
    return *this->p;
}
