#include "inc/utils.h"

Utils::Utils(QObject *parent) : QObject(parent) {

}


template<typename T>
void Utils::printEigen(const T& mat, const string &msg) {

    std::cout << msg << std::endl << " " << mat << std::endl;
}

template void Utils::printEigen<VectorXd>(const VectorXd &vec, const string &msg = "");
template void Utils::printEigen<VectorXi>(const VectorXi &vec, const string &msg = "");
template void Utils::printEigen<MatrixXd>(const MatrixXd &vec, const string &msg = "");
template void Utils::printEigen<MatrixXi>(const MatrixXi &vec,  const string &msg = "");
template void Utils::printEigen<ArrayXi> (const ArrayXi &vec,  const string &msg = "");


template<typename T>
void Utils::print(const vector<T> &vec, const string &msg) {

    std::cout << msg << " ";
    for(auto v : vec) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
}

template void Utils::print<int>(const vector<int> &vec, const string &msg = "");
template void Utils::print<double>(const vector<double> &vec, const string &msg = "");


template<typename T>
void Utils::print(const set<T> &vec, const string &msg) {

    std::cout << msg << " ";
    for(auto v : vec) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
}

template void Utils::print<int>(const set<int> &vec, const string &msg = "");


int Utils::printf(const char *fmt, ...) {

    char buffer[4096];
    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    std::cout << buffer << std::endl;
    return rc;
}


shared_ptr<vector<double>> Utils::getWithIndices(const vector<double> &vec, const vector<int> &indices) {

    shared_ptr<vector<double>> result = make_shared<vector<double>>();
    // result->resize(indices.size());

    for (auto index : indices)
        result->push_back(vec[index]);

    return result;
}

shared_ptr<VectorXd> Utils::getWithIndices(const VectorXd &vec, const vector<int> &indices) {

    VectorXd result = VectorXd::Zero(indices.size(), 1);

    for(uint i = 0; i < indices.size(); i++) {
        result(i, 0) = vec( indices[i], 0);
    }
//    for (auto index : indices)
//        result() ->push_back(vec[index]);

    return make_shared<VectorXd>( result );
}


PtrVecState Utils::getWithIndices(const VecState &vec, const vector<int> &indices) {

    PtrVecState result = make_shared<VecState>();
    // result->reserve(indices.size());

    for (auto index : indices)
        result->push_back( make_shared<State>( *vec[index] ) );

    return result;
}

shared_ptr<ArrayXi> Utils::setToArrayXi(const set<int> &s) {

    vector<int> vec(s.begin(), s.end());
    ArrayXi indices = Eigen::Map<ArrayXi, Eigen::Unaligned>(vec.data(), vec.size());
    return make_shared<ArrayXi>( indices );
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

std::string Utils::vectorToStr(const VectorXd& vec) {

    std::stringstream ss;
    ss << vec;
    return ss.str();
}

std::string Utils::matrixToStr(const MatrixXd& mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str();
}




