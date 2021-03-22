#ifndef MULTIHYPOTHESIS_H
#define MULTIHYPOTHESIS_H

#include <QObject>

#include "singlehypothesis.h"

class MultiHypothesis : public QObject
{
    Q_OBJECT
public:
    explicit MultiHypothesis(QList<SingleHypothesis *> *items, QObject *parent = nullptr);

    QList<SingleHypothesis *> *items;

    friend class TestMultiHypothesis;

protected:
    void normalizeWeights();

signals:

};

#endif // MULTIHYPOTHESIS_H
