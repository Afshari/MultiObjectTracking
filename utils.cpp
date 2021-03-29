#include "utils.h"

Utils::Utils(QObject *parent) : QObject(parent) {

}

MatrixXd Utils::blkdiag(const MatrixXd& a, int count) {

    MatrixXd bdm = MatrixXd::Zero(a.rows() * count, a.cols() * count);

    for (int i = 0; i < count; ++i)
        bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;

    return bdm;
}

VectorXd Utils::mean(MatrixXd *mat, VectorXd *weights) {

    VectorXd res(mat->rows());

    for(int i = 0; i < res.rows(); i++)
        res[i] = ( mat->row(i) * (*weights) ).sum();

    return res;
}

MatrixXd Utils::covar(QList<MatrixXd *> covars, MatrixXd *means, VectorXd *mean, VectorXd *weights) {

    MatrixXd *mMean = new MatrixXd(means->rows(), means->cols());
    for(int i = 0; i < means->cols(); i++)
        mMean->col(i) = *mean;

    MatrixXd *mWeights = new MatrixXd(covars[0]->rows(), weights->rows());

    for(int i = 0; i < mWeights->rows(); i++) {
        mWeights->row(i) = *weights;
    }

    MatrixXd deltaMeans = (*means) - (*mMean);


    MatrixXd* result = new MatrixXd( MatrixXd::Identity( covars.length(), covars[0]->rows() ) );

    for(int i = 0; i < result->rows(); i++) {
        MatrixXd mid = (*covars.at(i)).cwiseProduct((*mWeights));
        for(int j = 0; j < result->cols(); j++) {
            (*result)(i, j) = mid.row(j).sum();
        }
    }

    *result = (*result) + (*mWeights).cwiseProduct(deltaMeans) * deltaMeans.transpose();

    return *result;
}






