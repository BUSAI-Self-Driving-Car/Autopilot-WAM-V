#ifndef UTILITY_CONVERT_YYAMLEIGEN_H
#define UTILITY_CONVERT_YYAMLEIGEN_H


#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace YAML {

template<>
struct convert<Eigen::MatrixXd> {
    static Node encode(const Eigen::MatrixXd& rhs) {
        Node node;

        for (int row = 0; row < rhs.rows(); ++row)
        {
            Node row_node;
            for (int col = 0; col < rhs.cols(); ++col )
            {
                row_node.push_back(rhs(row, col));
            }
            node.push_back(row_node);
        }

        return node;
    }

    static bool decode(const Node& node, Eigen::MatrixXd& rhs) {
        if(!node.IsSequence() || node.size() == 0) {
            return false;
        }

        int rows = node.size();
        int cols = node[0].size();

        rhs.resize(rows,cols);

        for (int row=0; row < rows; ++row)
        {
            Node row_node = node[row];
            for (int col = 0; col < cols; ++col)
            {
                rhs(row,col) = row_node[col].as<double>();
            }
        }

        return true;
    }
};

template<>
struct convert<Eigen::VectorXd> {
    static Node encode(const Eigen::VectorXd& rhs) {
        Node node;
        for (int row = 0; row < rhs.size(); ++row)
        {
            node.push_back(rhs(row));
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::VectorXd& rhs) {
        if(!node.IsSequence() || node.size() == 0) {
            return false;
        }

        int rows = node.size();
        rhs.resize(rows);
        for (int row=0; row < rows; ++row)
        {
            rhs(row) = node[row].as<double>();
        }

        return true;
    }
};


}

#endif // UTILITY_CONVERT_YYAMLEIGEN_H

