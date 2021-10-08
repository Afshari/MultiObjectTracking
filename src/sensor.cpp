#include "inc/sensor.h"

Sensor::Sensor(double P_D, double lambda_c, MatrixXd range_c, QObject *parent) : QObject(parent) {

    this->P_D = P_D;
    this->lambda_c = lambda_c;
    this->range_c = range_c;

    double V = ( range_c(0, 1) - range_c(0, 0) ) * ( range_c(1, 1) - range_c(1, 0) );
    this->pdf_c = 1 / V;
    this->intensity = lambda_c / V;
}


double Sensor::get_P_D() {

    return this->P_D;
}

double Sensor::get_lambda_c() {

    return this->lambda_c;
}

double Sensor::get_pdf_c() {

    return this->pdf_c;
}

double Sensor::get_intensity() {

    return this->intensity;
}

MatrixXd Sensor::get_range_c() {

    return this->range_c;
}





