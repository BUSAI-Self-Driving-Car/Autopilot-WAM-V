#include "DStarLite.h"
#include <limits>
#include <stdexcept>

DStarLite::DStarLite()
    : initialised(false)
    , diagCost(std::sqrt(2))
    , adjCost(1)
    , obsCost(std::numeric_limits<double>::infinity())
{ }

void DStarLite::init(uint rows_, uint cols_)
{
    rows = rows_;
    cols = cols_;
    rhs.resize(rows, cols);
    g.resize(rows, cols);
    openList.resize(rows, cols);
    listCost.resize(rows, cols);

    initialised = true;
}

DStarLite::Path DStarLite::operator () (const Eigen::SparseMatrix<int>& occupied, const Eigen::Vector2i& startIdx, const Eigen::Vector2i& goalIdx)
{
    if (!initialised) {
        throw std::runtime_error("DStar not initialised");
    }

    goal = goalIdx;
    rhs.setConstant(rows, cols, std::numeric_limits<double>::infinity());
    g.setConstant(rows, cols,std::numeric_limits<double>::infinity());
    openList.setZero(rows, cols);
    listCost.setConstant(rows, cols, std::numeric_limits<double>::infinity());

    rhs(goalIdx[0],goalIdx[1]) = 0;
    listCost(goalIdx[0],goalIdx[1]) = 0;

    while ( listCost.minCoeff() < calculateKey(startIdx[0], startIdx[1]) || g(startIdx[0], startIdx[1]) != rhs(startIdx[0], startIdx[1])) {

        int x,y;
        listCost.minCoeff(&x, &y);
        openList(x,y) = 0;
        listCost(x,y) = std::numeric_limits<double>::infinity();

        if (g(x,y) > rhs(x,y)) {
            g(x,y) = rhs(x,y);
            updateNeighbours(x,y, occupied);
        }
        else {
            g(x,y) = std::numeric_limits<double>::infinity();
            updateNeighbours(x,y, occupied);
            updateVertex(x,y, occupied);
        }
    }

    Eigen::Vector2i point(startIdx);
    Path path;
    path.conservativeResize(2,1);
    path.block<2,1>(0,0) << startIdx;
    int segments = 0;

    while ( point[0] != goal[0] || point[1] != goal[1] ) {
        ++segments;
        double cost = std::numeric_limits<double>::infinity();
        int x = point[0];
        int y = point[1];

        for (int i = x-1; i <= x+1; ++i) {
            for (int j = y-1; j <= y+1; ++j) {

                if (i >= 0 && j >= 0 && i < rows && j < cols ) {
                    if (g(i,j) < cost) {
                        cost = g(i,j);
                        path.conservativeResize(2,segments+1);
                        path.block<2,1>(0,segments) << i, j;
                    }
                }
            }
        }
        point = path.block<2,1>(0, segments);
    }

    return path;
}

void DStarLite::updateNeighbours(int x, int y, const Eigen::SparseMatrix<int>& occupied)
{
    for (int in = x-1; in <= x+1; ++in) {

        for (int jn = y-1; jn <= y+1; ++jn) {

            if ( in >= 0 && in < rows && jn >= 0 && jn < cols && (in != x || jn != y) ) {
                updateVertex(in, jn, occupied);
            }
        }
    }
}

void DStarLite::updateVertex(int x, int y, const Eigen::SparseMatrix<int>& occupied)
{
    if (goal[0] != x || goal[1] != y) {

        double temp = std::numeric_limits<double>::infinity();
        double transitionCost = std::numeric_limits<double>::infinity();

        for (int iv = x-1; iv <= x+1; ++iv) {

            for (int jv = y-1; jv <= y+1; ++jv) {

                if ( iv >= 0 && iv < rows && jv >= 0 && jv < cols && (iv != x || jv != y) ) {

                   if (occupied.coeff(iv,jv) > 0 || occupied.coeff(x,y) > 0) {
                       transitionCost = obsCost;
                   }
                   else if (iv == x || jv == y ) {
                       transitionCost = adjCost;
                   }
                   else {
                       transitionCost = diagCost;
                   }
                   temp = std::min(temp,g(iv,jv)+transitionCost);
                }
            }
        }

        rhs(x,y) = temp;
    }

    if (g(x,y) != rhs(x,y)) {
        openList(x,y) = 1;
        listCost(x,y) = std::min(g(x,y),rhs(x,y));
    }
    else {
        openList(x,y) = 0;
        listCost(x,y) = std::numeric_limits<double>::infinity();
    }
}

void DStarLite::addToList(uint x, uint y)
{
    openList(x,y) = 1;
    listCost(x,y) = calculateKey(x,y);
}

double DStarLite::calculateKey(uint x, uint y)
{
    double key = std::min(g(x,y), rhs(x,y));
    return key;
}
