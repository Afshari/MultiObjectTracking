#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <QObject>

class MeasurementModel : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementModel(QObject *parent = nullptr);

signals:

};

#endif // MEASUREMENTMODEL_H
