#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <QObject>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::tuple;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Hungarian : public QObject {
    Q_OBJECT
public:
    explicit Hungarian(QObject *parent = nullptr);

    double Solve(vector<vector<double>> &DistMatrix, vector<int> &Assignment);
    tuple<vector<int>, double> Solve(const MatrixXd &L);

private:
    void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix,
                bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix,
                bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix,
               bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix,
               bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix,
               bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

};


#endif // HUNGARIAN_H
