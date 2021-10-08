#include "inc/input_parser.h"

InputParser::InputParser(QObject *parent) : QObject(parent) {

}

int InputParser::getCode(const string& data) {

    string delimiter = ":";

    size_t pos = data.find(delimiter);
    if(pos > data.length())
        return -1;

    string token = data.substr(0, pos);
    return std::stoi(token);
}


shared_ptr<vector<int>> InputParser::getIndices(const string& data, const string& delimiter) {

    vector<int> indices;

    int pos = 0;
    indices.push_back( pos );
    while(true) {
        pos = data.find(delimiter, pos);
        if(pos == -1)
            break;
        pos += 1;
        indices.push_back( pos );
    }

    return make_shared<vector<int>>( indices );
}



shared_ptr<MatrixXd> InputParser::getMeasurements(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ";");

    MatrixXd measurements(2, indices->size());

    string currMeasurement = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            currMeasurement = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            currMeasurement = token.substr( indices->at(i), token.length() );
        }
        auto idx = getIndices(currMeasurement, ",");
        float x = std::stof( currMeasurement.substr( (*idx)[0], (*idx)[1] - (*idx)[0] - 1 ) );
        float y = std::stof( currMeasurement.substr( (*idx)[1], currMeasurement.length() - (*idx)[1] ) );

        measurements(0, i) = x;
        measurements(1, i) = y;
    }

    return make_shared<MatrixXd>( measurements );
}

shared_ptr<VectorXd> InputParser::getOneVectorX(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ",");

    VectorXd state(indices->size());

    string currState = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            currState = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            currState = token.substr( indices->at(i), token.length() );
        }
        float x = std::stof( currState );

        state(i, 0) = x;
    }

    return make_shared<VectorXd>( state );
}


shared_ptr<VectorXi> InputParser::getOneVectorXi(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ",");

    VectorXi state(indices->size());

    string currState = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            currState = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            currState = token.substr( indices->at(i), token.length() );
        }
        int x = std::stoi( currState );

        state(i, 0) = x;
    }

    return make_shared<VectorXi>( state );
}


shared_ptr<MatrixXd> InputParser::getOneMatrixX(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ";");

    string firstRow = token.substr( indices->at(0), indices->at(1) - indices->at(0) -1 );
    int numCols = getIndices(firstRow, ",")->size();
    MatrixXd result = MatrixXd::Zero(indices->size(), numCols);
    string curr = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            curr = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            curr = token.substr( indices->at(i), token.length() );
        }

        VectorXd x = *this->getOneVectorX(curr, 0, curr.length());
        result(i, Eigen::all) = x;
    }

    return make_shared<MatrixXd>( result );
}


shared_ptr<MatrixXi> InputParser::getOneMatrixXi(const string& data, int start_index, int len) {

    string token = data.substr(start_index, len);
    auto indices = getIndices(token, ";");

    string firstRow = token.substr( indices->at(0), indices->at(1) - indices->at(0) -1 );
    int numCols = getIndices(firstRow, ",")->size();
    MatrixXi result = MatrixXi::Zero(indices->size(), numCols);
    string curr = "";
    for(auto i = 0U; i < indices->size(); i++) {

        if(i < indices->size() - 1) {
            curr = token.substr( indices->at(i), indices->at(i + 1) - indices->at(i) - 1 );
        } else {
            curr = token.substr( indices->at(i), token.length() );
        }

        VectorXi x = *this->getOneVectorXi(curr, 0, curr.length());
        result(i, Eigen::all) = x;
    }

    return make_shared<MatrixXi>( result );
}



PtrVecState InputParser::getStates(const string &strX, const string &strP) {

    auto indicesX = getIndices(strX, ";");
    auto indicesP = getIndices(strP, ";");

    // Utils::printf("str x %s", strX.c_str());
    // Utils::printf("str P %s", strP.c_str());

    string currX = "";
    string currP = "";

    VecState states;

    for(auto i = 0U; i < indicesX->size(); i++) {

        if(i < indicesX->size() - 1) {
            currX = strX.substr( indicesX->at(i), indicesX->at(i + 1) - indicesX->at(i) - 1 );
            currP = strP.substr( indicesP->at(i), indicesP->at(i + 1) - indicesP->at(i) - 1 );
        } else {
            currX = strX.substr( indicesX->at(i), strX.length() );
            currP = strP.substr( indicesP->at(i), strP.length() );
        }

        states.push_back( getState(currX, currP) );
    }

    return make_shared<VecState>( states );
}


NestedPtrVecState InputParser::getNestedStates(const string &strX, const string &strP) {

    auto indicesX = getIndices(strX, "|");
    auto indicesP = getIndices(strP, "|");

    vector<PtrVecState> allStates;

    string currX = "";
    string currP = "";

    for(auto i = 0U; i < indicesX->size(); i++) {

        if(i < indicesX->size() - 1) {
            currX = strX.substr( indicesX->at(i), indicesX->at(i + 1) - indicesX->at(i) - 1 );
            currP = strP.substr( indicesP->at(i), indicesP->at(i + 1) - indicesP->at(i) - 1 );
        } else {
            currX = strX.substr( indicesX->at(i), strX.length() );
            currP = strP.substr( indicesP->at(i), strP.length() );
        }

        PtrVecState currStates = getStates(currX, currP);
        allStates.push_back( currStates );
//        Utils::printf("index %d", i);
//        for(auto state : *currStates) {
//            Utils::printEigen<VectorXd>(state->getX(), "x");
//        }
//        Utils::printf("--------------------");
    }

    return make_shared<vector<PtrVecState>>( allStates );
}


shared_ptr<State> InputParser::getState(const string &strX, const string &strP) {

    auto indicesX = getIndices(strX, ",");
    auto indicesP = getIndices(strP, ",");


    VectorXd x(indicesX->size());
    MatrixXd P(indicesX->size(), indicesX->size());

    string currX = "";
    for(auto i = 0U; i < x.rows(); i++) {

        if(i < indicesX->size() - 1) {
            currX = strX.substr( indicesX->at(i), indicesX->at(i + 1) - indicesX->at(i) - 1 );
        } else {
            currX = strX.substr( indicesX->at(i), strX.length() );
        }
        x(i, 0) = std::stof( currX );
    }

    string currP = "";
    for(auto i = 0U; i < indicesP->size(); i++) {

        if(i < indicesP->size() - 1) {
            currP = strP.substr( indicesP->at(i), indicesP->at(i + 1) - indicesP->at(i) - 1 );
        } else {
            currP = strP.substr( indicesP->at(i), strP.length() );
        }
        P(int(i/P.cols()), i%P.cols()) = std::stof( currP );
    }

    return make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );
}










