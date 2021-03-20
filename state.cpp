#include "state.h"

State::State(QObject *parent) : QObject(parent) {

}

VectorXd State::getX() const {
    return *this->x;
}

MatrixXd State::getP() const {
    return *this->p;
}
