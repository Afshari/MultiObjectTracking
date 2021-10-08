#ifndef MURTY_STATE_H
#define MURTY_STATE_H

#include <QObject>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include "libs/murty/murty_funcs.h"

class MurtyState : public QObject {
    Q_OBJECT

public:
    using Slacklist = std::vector<std::tuple<double, unsigned, unsigned>>;
    Eigen::MatrixXd C;
    Slack u, v;
    double cost;
    double boundcost;
    bool solved;
    Assignment solution;
    Assignment res;
    std::vector<unsigned> rmap, cmap;

public:
    explicit MurtyState(const MurtyState* s);

    explicit MurtyState(const Eigen::MatrixXd& C_);

    auto partition_with(const unsigned i, const unsigned j)
    {
        auto s = std::make_shared<MurtyState>(this);

        allbut(C, s->C, i, j);
        allbut(u, s->u, i);
        allbut(v, s->v, j);
        //std::cout << "            Binding (" << rmap[i] << "," << cmap[j] << ") (" << C(i, j) << ") [" << s->C.rows() << "x" << s->C.cols() << "]" << std::endl;
        s->rmap.erase(s->rmap.begin() + i);
        s->cmap.erase(s->cmap.begin() + j);
        s->boundcost += C(i, j);

        return s;
    }

    auto partition_without(const unsigned i, const unsigned j, const double slack)
    {
        auto s = std::make_shared<MurtyState>(this);
        s->C = C;
        s->u = u;
        s->v = v;
        s->remove(i, j, slack);

        return s;
    }

    void remove(const unsigned i, const unsigned j, const double slack);

    bool solve();

    MurtyState::Slacklist minslack() const;

    void allbut(const Eigen::MatrixXd& from, Eigen::MatrixXd& to, const unsigned row, const unsigned col);
    void allbut(const Eigen::Matrix<double, Eigen::Dynamic, 1>& from, Eigen::Matrix<double, Eigen::Dynamic, 1>& to, const unsigned row);
    void allbut(const Eigen::Matrix<double, 1, Eigen::Dynamic>& from, Eigen::Matrix<double, 1, Eigen::Dynamic>& to, const unsigned col);

};

using MurtyStatePtr = std::shared_ptr<MurtyState>;
struct MurtyStatePtrCompare {
    bool operator()(const MurtyStatePtr& a, const MurtyStatePtr& b) {
        if (a->cost > b->cost) {
            return true;
        } else if (a->cost == b->cost) {
            return a->C.rows() > b->C.rows();
        } else {
            return false;
        }
    }
};

#endif // MURTY_STATE_H
