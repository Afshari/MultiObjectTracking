#include "pda.h"

PDA::PDA(Estimator *estimator, float spatialClutterIntensity, float P_D , float P_G) :
    Associator(estimator, spatialClutterIntensity, P_D, P_G) {

}

QList<MultiHypothesis *> *PDA::associate(const QList<State *> &objects, const QList<Detection *> &detections, int dt) {

    QList<MultiHypothesis *> *lst = new QList<MultiHypothesis *>();
    for(int i = 0; i < objects.length(); i++) {
        lst->append(hypothesis(objects.at(i), detections, dt));
    }

    return lst;
}

MultiHypothesis *PDA::hypothesis(State *prior, const QList<Detection *> &detections, int dt) {

//    StateGaussian prior(recvX, recvP);
    State *predicted = estimator->predict(*prior, dt);
    MeasurementPrediction *meas = estimator->predictMeasurement(predicted);

    QList<SingleHypothesis *> *list = new QList<SingleHypothesis *>();
    Detection *missDetection = new Detection(Detection::DetectionType::miss);
    SingleHypothesis *single = new SingleHypothesis(missDetection, predicted,
                                                    this->estimator->transitionModel,
                                                    nullptr, 1 - this->P_D * this->P_G);
    list->append(single);


//    std::cout << "x Pred: \r\n" << meas->statePred->getX() << std::endl;
//    std::cout << "predic: \r\n" << predicted->getX() << std::endl;

    for(int i = 0; i < detections.length(); i++) {

        double log_pdf = this->logPDF(*detections[i]->x, *meas->zPred, *meas->S);
        double probability = this->getProbability(log_pdf);
//                std::cout << probability << std::endl;
        SingleHypothesis *single = new SingleHypothesis(detections[i], predicted,
                                                       this->estimator->transitionModel,
                                                       meas, probability);
//        std::cout << "S: \r\n" << *meas->S << std::endl;
//        std::cout << "Upsilon: \r\n" << *meas->upsilon << std::endl;
        list->append(single);
//                std::cout << *(*recvMeasurements)[i] << std::endl;
    }

    MultiHypothesis *multi = new MultiHypothesis(list);
    multi->normalizeWeights();

    return multi;
}

double PDA::logPDF(const VectorXd &xDetection, const VectorXd &xMeasPred, const MatrixXd &PMeasPred) {

    VectorXd mean(2);
    mean << 0, 0;

    return stats::dmvnorm<VectorXd, MatrixXd>((xDetection - xMeasPred), mean, PMeasPred, true);
}

double PDA::toPDF(const double &logPDF) {
    return qExp(logPDF);
}

double PDA::getProbability(const double &logPDF) {

    double pdf = toPDF(logPDF);
    return (pdf * P_D) / spatialClutterIntensity;
}
