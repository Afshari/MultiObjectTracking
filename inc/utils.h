#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <Eigen/Dense>
#include <iostream>
#include <stdarg.h>
#include <set>

#include "inc/state.h"

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::ArrayXi;
using Eigen::Vector;
using std::tuple;
using std::shared_ptr;
using std::make_shared;
using std::string;
using std::vector;
using std::set;

class Utils : public QObject {
    Q_OBJECT
public:
    explicit Utils(QObject *parent = nullptr);

    static MatrixXd blkdiag(const MatrixXd& a, int count);
    static VectorXd mean(MatrixXd *means, VectorXd *weights);
    static MatrixXd covar(QList<MatrixXd *> covars, MatrixXd *means, VectorXd *mean, VectorXd *weights);

    static std::string matrixToStr(const Eigen::MatrixXd& mat);
    static std::string vectorToStr(const Eigen::VectorXd& vec);

    static int  printf(const char *fmt, ...);

    template<typename T>
    static void printEigen(const T& mat, const string &msg = "");

    template<typename T>
    static void print(const vector<T> &vec, const string &msg = "");

    template<typename T>
    static void print(const set<T> &vec, const string &msg = "");

    static shared_ptr<ArrayXi> setToArrayXi(const set<int> &s);


    static shared_ptr<VectorXd> getWithIndices(const VectorXd &vec, const vector<int> &indices);
    static shared_ptr<vector<double>> getWithIndices(const vector<double> &vec, const vector<int> &indices);
    static PtrVecState getWithIndices(const VecState &vec, const vector<int> &indices);

};



#endif // UTILS_H
