#include "inc/pdatracker.h"

PDATracker::PDATracker(QObject *parent) : QObject(parent) {

    MatrixXd *mNoiseCovar = new MatrixXd(0.75 * MatrixXd::Identity(2, 2));
    measurementModel = new MeasurementLinearGaussian(mNoiseCovar);


    transitionLinearGaussian = new TransitionLinearGaussian(0.005);
    kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
    pda = new PDA(kalman, 0.125, 0.9);

}

void PDATracker::initialize() {

    dt = 0;
//    scale = 1;
}

void PDATracker::setScale(int scale) {
    this->scale = scale;
}

void PDATracker::setX(Eigen::VectorXd *X) {
    this->X = X;
}

Eigen::VectorXd *PDATracker::getX() {
    return this->X;
}

void PDATracker::setP(Eigen::MatrixXd *P) {
    this->P = P;
}

Eigen::MatrixXd *PDATracker::getP() {
    return this->P;
}

StateGaussian *PDATracker::getCurrentState() {
    return new StateGaussian(X, P);
}

void PDATracker::setMeasurements(QList<Detection *> *measurements) {
    this->measurements = measurements;
}

QList<Detection *> *PDATracker::getMeasurements() {
    return measurements;
}

void PDATracker::interpretX(QStringList receivedArr) {

    QStringList dim = receivedArr[1].split(',');
    QStringList items = receivedArr[2].split(',');

    VectorXd predState(dim[0].toUInt());
    for(int i = 0; i < predState.size(); i++) {
        predState[i] = items[i].toFloat() / this->scale;
    }

    setX(new VectorXd(predState));

    std::cout << predState;
}


void PDATracker::interpretP(QStringList receivedArr) {

    QStringList dim = receivedArr[1].split(',');
    QStringList items = receivedArr[2].split(',');

    MatrixXd predCov(dim[0].toUInt(), dim[1].toUInt());
    for(int i = 0; i < predCov.rows(); i++) {
        for(int j = 0; j < predCov.cols(); j++) {
            predCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
        }
    }

    setP(new MatrixXd(predCov));

    std::cout << predCov;
}

void PDATracker::interpretMeasurements(QStringList receivedArr) {

    measurements = new QList<Detection *>();
    for(int i = 1; i < receivedArr.size(); i+=3) {
        if(receivedArr[i].toUInt() == 23) {

            QStringList dim = receivedArr[i+1].split(',');
            QStringList items = receivedArr[i+2].split(',');

            Detection *detection = new Detection(Detection::DetectionType::detect);
            VectorXd *measurement = new VectorXd(dim[0].toUInt());
            for(int j = 0; j < (*measurement).size(); j++) {
                (*measurement)[j] = items[j].toFloat() / this->scale;
            }
            detection->x = measurement;
            measurements->append(detection);
        }
    }
}

QString PDATracker::trackLoop() {

    QList<State *> objects;
    objects.append(getCurrentState());

    QList<MultiHypothesis *> *res = pda->associate(objects, *getMeasurements(), dt);

    QList<State *> states;

    QList<double> weights;
    for(int i = 0; i < res->length(); i++) {
        QList<SingleHypothesis *> *currItems = res->at(i)->items;

        QString dataToSend = "";
        for(int j = 0; j < currItems->length(); j++) {
            QString currData = "";

            SingleHypothesis *currHypothesis = currItems->at(j);

            if(currHypothesis->detection->type == Detection::DetectionType::detect) {

                State* posteriorState = kalman->update(
                            *currHypothesis->state,
                            *measurementModel,
                            *currHypothesis->detection->x,
                            *currHypothesis->measurementPrediction);

                states.append(posteriorState);

            } else if(currHypothesis->detection->type == Detection::DetectionType::miss) {
                states.append(currHypothesis->state);
            }
            weights.append(currHypothesis->probability);
        }

    }

    VectorXd vecWeights(weights.length());
    for(int i = 0; i < weights.length(); i++)
        vecWeights(i) = weights[i];
    MatrixXd matMean(states[0]->getX().rows(), states.length());

    QList<MatrixXd *> covars;
    for(int i = 0; i < matMean.rows(); i++) {
        covars.append(new MatrixXd(MatrixXd::Zero(matMean.rows(), matMean.cols())));
    }

    for(int i = 0; i < matMean.cols(); i++) {
        matMean.col(i) = states[i]->getX();
        for(int j = 0; j < states[i]->getP().cols(); j++) {
            covars.at(j)->col(i) = states[i]->getP().row(j);
        }
    }

    VectorXd posteriorMean = Utils::mean(&matMean, &vecWeights);
    setX(new VectorXd(posteriorMean));

    MatrixXd posteriorCovar = Utils::covar(covars, &matMean, &posteriorMean, &vecWeights);
    setP(new MatrixXd(posteriorCovar));

    QString response = QString("%1 | %2").arg(
        QString::fromStdString(Utils::vectorToStr(posteriorMean * this->scale)),
        QString::fromStdString(Utils::matrixToStr(posteriorCovar)));

    dt = 1;

    return response;
}






