#include "libs/murty/murty_state.h"

MurtyState::MurtyState(const MurtyState* s) :
    cost(s->cost), boundcost(s->boundcost), solved(false),
    solution(s->solution), rmap(s->rmap), cmap(s->cmap) {

}

MurtyState::MurtyState(const Eigen::MatrixXd& C_) :
      C(C_), u(C_.rows()), v(C_.cols()), cost(0),
      boundcost(0), solved(false), solution(C_.rows()) {
        v.setZero();
        rmap.resize(C.rows());
        cmap.resize(C.cols());
        for (unsigned i = 0; i < C.rows(); ++i) { rmap[i] = i; }
        for (unsigned j = 0; j < C.cols(); ++j) { cmap[j] = j; }
}


void MurtyState::remove(const unsigned i, const unsigned j, const double slack) {
    //std::cout << "            Removing (" << rmap[i] << "," << cmap[j] << ") (" << C(i, j) << ") [" << C.rows() << "x" << C.cols() << "] raising cost " << cost << " -> ";
    C(i, j) = lap_inf;
    solved = false;
    cost += slack;
    //std::cout << cost << std::endl;
    //std::cout << "New C: " << std::endl << C << std::endl;
}

bool MurtyState::solve() {
    //std::cout << "Solving (" << cost << "): " << std::endl << C << std::endl;
    res.resize(C.rows());
    lap(C, res, u, v);
    //std::cout << "rmap: ";
    //for (auto& r : rmap) {
        //std::cout << r << ", ";
    //}
    //std::cout << std::endl;
    //std::cout << "cmap: ";
    //for (auto& c : cmap) {
        //std::cout << c << ", ";
    //}
    //std::cout << std::endl;
    cost = boundcost;
    for (unsigned i = 0; i < res.rows(); ++i) {
        solution[rmap[i]] = cmap[res[i]];
        cost += C(i, res[i]);
    }
    //std::cout << "Solution: [" << res.transpose() << "] " << cost << std::endl;
    solved = true;
    return (cost < lap_inf);
}

MurtyState::Slacklist MurtyState::minslack() const {
    std::vector<std::tuple<double, unsigned, unsigned>> mslack(C.rows());
    double h;
    for (unsigned i = 0; i < C.rows(); ++i) {
        mslack[i] = {C(i, 0) - u[i] - v[0], i, res[i]};
        for (unsigned j = 1; j < C.cols(); ++j) {
            if (static_cast<int>(j) == res[i]) {
                continue;
            }

            h = C(i, j) - u[i] - v[j];
            if (h < std::get<0>(mslack[i])) {
                mslack[i] = {h, i, res[i]};
            }
        }
    }
    std::sort(mslack.rbegin(), mslack.rend());
    return mslack;
}

void MurtyState::allbut(const Eigen::MatrixXd& from, Eigen::MatrixXd& to, const unsigned row, const unsigned col)
{
    unsigned rows = from.rows();
    unsigned cols = from.cols();

    to.resize(rows - 1, cols - 1);

    if (row > 0 && col > 0) {
        to.block(0, 0, row, col) = from.block(0, 0, row, col);
    }
    if (row > 0 && col < cols - 1) {
        to.block(0, col, row, cols - col - 1) =
            from.block(0, col + 1, row, cols - col - 1);
    }
    if (row < rows - 1 && col > 0) {
        to.block(row, 0, rows - row - 1, col) =
            from.block(row + 1, 0, rows - row - 1, col);
    }
    if (row < rows - 1 && col < cols - 1) {
        to.block(row, col, rows - row - 1, cols - col - 1) =
            from.block(row + 1, col + 1, rows - row - 1, cols - col - 1);
    }
}

void MurtyState::allbut(const Eigen::Matrix<double, Eigen::Dynamic, 1>& from, Eigen::Matrix<double, Eigen::Dynamic, 1>& to, const unsigned row) {
    unsigned rows = from.rows();

    to.resize(rows - 1, 1);

    if (row > 0) {
        to.block(0, 0, row, 1) = from.block(0, 0, row, 1);
    }
    if (row < rows - 1) {
        to.block(row, 0, rows - row - 1, 1) =
            from.block(row + 1, 0, rows - row - 1, 1);
    }
}

void MurtyState::allbut(const Eigen::Matrix<double, 1, Eigen::Dynamic>& from, Eigen::Matrix<double, 1, Eigen::Dynamic>& to, const unsigned col) {
    unsigned cols = from.cols();

    to.resize(1, cols - 1);

    if (col > 0) {
        to.block(0, 0, 1, col) = from.block(0, 0, 1, col);
    }
    if (col < cols - 1) {
        to.block(0, col, 1, cols - col - 1) =
            from.block(0, col + 1, 1, cols - col - 1);
    }
}


















