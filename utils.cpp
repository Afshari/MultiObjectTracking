#include "utils.h"

Utils::Utils(QObject *parent) : QObject(parent)
{

}

MatrixXd Utils::blkdiag(const MatrixXd& a, int count) {

    MatrixXd bdm = MatrixXd::Zero(a.rows() * count, a.cols() * count);

    for (int i = 0; i < count; ++i)
        bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;

    return bdm;
}
