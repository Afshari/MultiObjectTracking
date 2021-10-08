#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include <QObject>
#include <Eigen/Dense>

using std::shared_ptr;
using std::make_shared;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementModel : public QObject {
    Q_OBJECT
public:
    explicit MeasurementModel(QObject *parent = nullptr);
    virtual shared_ptr<MatrixXd> H(const VectorXd &x) { std::ignore = x; return nullptr; }
    virtual shared_ptr<VectorXd> h(const VectorXd &state) { std::ignore = state; return nullptr; }

    virtual MatrixXd S(const MatrixXd &upsilon) { std::ignore = upsilon; return MatrixXd(); }
    virtual MatrixXd upsilon(const MatrixXd &predCov) { std::ignore = predCov; return MatrixXd(); }

    virtual MatrixXd getR();

protected:
    MatrixXd* measurementNoiseCovariance;

    shared_ptr<MatrixXd>  R;
    shared_ptr<MatrixXd> _H;
    float d;

};

#endif // MEASUREMENT_MODEL_H
