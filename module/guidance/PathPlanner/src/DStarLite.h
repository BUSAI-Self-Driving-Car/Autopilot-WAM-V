#ifndef DSTARLITE_H
#define DSTARLITE_H

#include <Eigen/Core>
#include <Eigen/Sparse>

class DStarLite
{
public:
    typedef Eigen::Matrix<int, 2, Eigen::Dynamic> Path;

    DStarLite();

    Path operator () (const Eigen::SparseMatrix<int>& occupied, const Eigen::Vector2i& startIdx, const Eigen::Vector2i& goalIdx);

    void init(uint rows_, uint cols_);

private:
    bool initialised;
    uint rows;
    uint cols;
    double diagCost;
    double adjCost;
    double obsCost;
    Eigen::Vector2i goal;
    Eigen::MatrixXd rhs;
    Eigen::MatrixXd g;
    Eigen::MatrixXd openList;
    Eigen::MatrixXd listCost;


   void addToList(uint x, uint y);

   double calculateKey(uint x, uint y);

   void updateVertex(int x, int y, const Eigen::SparseMatrix<int>& occupied);

   void updateNeighbours(int x, int y, const Eigen::SparseMatrix<int>& occupied);


};

#endif // DSTARLITE_H
