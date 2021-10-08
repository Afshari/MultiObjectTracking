#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include <QObject>
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include <map>

#include "inc/state.h"
#include "inc/utils.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;
using std::shared_ptr;
using std::make_shared;
using std::map;

class InputParser : public QObject {
    Q_OBJECT
public:
    explicit InputParser(QObject *parent = nullptr);

    virtual int getCode(const string& data);
    virtual shared_ptr<vector<int>> getIndices(const string& data, const string& delimiter);
    virtual shared_ptr<MatrixXd> getMeasurements(const string& data, int start_index, int len);
    virtual shared_ptr<VectorXd> getOneVectorX(const string& data, int start_index, int len);
    virtual shared_ptr<VectorXi> getOneVectorXi(const string& data, int start_index, int len);
    virtual shared_ptr<MatrixXd> getOneMatrixX(const string& data, int start_index, int len);
    virtual shared_ptr<MatrixXi> getOneMatrixXi(const string& data, int start_index, int len);
    virtual NestedPtrVecState getNestedStates(const string &strX, const string &strP);
    virtual PtrVecState getStates(const string &strX, const string &strP);
    virtual shared_ptr<State> getState(const string &strX, const string &strP);


};

#endif // INPUT_PARSER_H
